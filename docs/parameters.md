# The model and its parameters

The behaviour of the planner is set by four groups of numbers: the vehicle,
the control set, the state system, and the cost. We describe here what each
one is, what raising it does, and the reference configuration.

## Units

The planner works on a grid. `zoom` is how many grid units one metre of the
vehicle spans, so one grid unit is `1 / zoom` metres.

```python
env = from_image("data/maps/two_rooms.png", (100, 100), zoom=4)
```

The vehicle is 3.649 m long, so at `zoom=4` it covers 14.6 grid units and one
unit is 25 cm. **Every length, curvature and clearance the planner reports is in
grid units.** The single exception is the swept floor area, returned in square
metres, so that it can be read against the vehicle's own 2.55 m² footprint.
The notebooks and the tables of `figures/tables/` also call a grid unit a cell.

`zoom` fixes the size of the vehicle against the map, so it is part of the
problem, not a resolution setting. With the grid shape held fixed, raising
`zoom` makes the vehicle larger relative to the scene: passages narrow and some
close. To refine the grid on the same scene, scale the grid shape and `zoom`
together, as notebook 08 does; the cost is memory, in `nx · ny · n_theta · 4`.
The campaign scenes are run at zoom 8 on their 200-cell grids for Two-Rooms
and the narrow passage (zoom 4 on 100 cells, as in the example above, is the
same Two-Rooms at half the resolution; for the narrow passage that grid is too
coarse, see notebook 06 and § 5.2.4 of the thesis), 2 for the warehouse and
the procedural corridors, and 20 for the docking slot.

## The vehicle

`shfm.vehicle.Vehicle`, with `DEFAULT_VEHICLE` the forklift this package is
built around.

| parameter | default | meaning |
|---|---|---|
| `length` | 3.6490 m | total footprint, forks included |
| `width` | 0.7000 m | total footprint |
| `wheelbase` | 1.38 m | front axle to rear axle |
| `rear_axle_offset` | 0.4845 m | rear of the vehicle to the centre of the rear axle |
| `n_balls` | 12 | discs covering the footprint for collision checking |

The reference point is the **centre of the rear axle**, not the centre of the
envelope. That is the point the steering functions integrate, and it is why
`rear_axle_offset` exists: the footprint is not symmetric about the origin.

`n_balls` trades collision accuracy against the cost of the lift. The radius
is the smallest that lets the end discs reach the corners of the footprint, so
the covering is tight at the corners and overlaps between discs; the discs
reach past the sides and the ends, so the model never misses a collision and
slightly overestimates the vehicle in narrow passages. More discs make
the chain narrower but longer: with the default vehicle, 12 discs reach 10 cm
past each side and 17 cm past each end, 24 discs 3 cm and 23 cm. A passage
that closes with 12 discs can open with more, while a pose close to a wall,
end on, can turn into a collision. See `Vehicle.ball_radius` for the exact
expression.

## The control set

`shfm.kinematics.control_set` returns, for each sampled orientation, the
velocities the vehicle may follow in each of the four modes. The velocities
carry *speed coefficients*: dimensionless weights, not physical speeds, which
the thesis calls the coefficients ν of the modes. The vehicle is assumed to
travel at constant speed, and what the solver minimises is a cost of travel,
not a duration. Lengthening a vector lowers the cost of paths that use it,
which favours that motion.

The velocities are those of the centre of the rear axle. With front and rear
steering angles `delta_F` and `delta_R`, it moves along `theta + delta_R` and
the chassis turns at `v cos(delta_R) (tan delta_F - tan delta_R) / L`, `L`
being the wheelbase, which is what rolling without slip gives when `v` is the
speed of the rear-axle centre.
The factor `cos(delta_R)` only affects counter-phase turns: crab motion does
not rotate, and car-like motion has `delta_R = 0`.

| parameter | reference | what it weighs |
|---|---|---|
| `nu_fwd` | 3.0 | straight forward navigation |
| `nu_2ws` | 1.5 | 2WS turning, front axle only |
| `nu_4ws` | 0.5 | 4WS manoeuvring, both axles |
| `gamma_rev` | 0.5 | applied to the backward navigation mode |
| `delta_f_max` | 0.5 rad | front steering saturation |
| `delta_r_max` | 0.5 rad | rear steering saturation |

`REFERENCE_COEFFICIENTS` holds the first four. They satisfy the intended
ordering `nu_fwd > nu_2ws > nu_4ws`: straight forward navigation is preferred,
then 2WS turning, and manoeuvring only where the geometry requires it. With
another ordering the planner can crab down open corridors, which is what the
state system is meant to prevent.

`gamma_rev` lies in `[0, 1]`. Below one it discourages unnecessary reversing;
zero forbids it. It does not apply to the manoeuvring modes, where
reversing leaves the norm of the control unchanged.

The steering saturations of the table are the defaults of `control_set`. The
docking scene raises both to 1.05 rad, about 60 degrees, without which no
useful crab fits in the slot (notebooks 00, 04 and 05, and the docking campaign,
whose turning radius is sized on the same angle).

Two variants ship beside it:

* `control_set_2ws` locks the rear axle (`delta_r_max = 0`): the same planner
  with car-like kinematics. This is the baseline the 4WS results are compared
  against.
* `control_set_unstructured` gives the eight extreme steering vectors in a
  single state, with no mode structure and no cost for switching.
  This is what the state system improves on. Its own defaults differ: both
  saturations at 1.05 rad and every control at unit speed.

## The state system

The state system has four modes and a cost for changing mode.
`shfm.states.transition_matrix(k1, k2)` builds the matrix; never write the
array by hand, because two index conventions coexist (see `SOLVER_ORDER` and
`THESIS_ORDER`).

| parameter | charged for |
|---|---|
| `k1` | reversing the direction of travel while staying in navigation |
| `k2` | entering or leaving the manoeuvring mode |
| (fixed, 1) | reversing direction *within* the manoeuvring mode |

The admissible range is `1 <= k1 < k2`, and `transition_matrix` refuses anything
else: reversing must be cheaper than engaging the manoeuvring mode, and no
transition may be cheaper than the reference unit.

Each entry is a constant scalar, independent of velocity and steering angle. The
solver scales it by the local environment cost, so a transition is priced in the
same units as motion at that point.

**No entry should be zero.** The solver squares and inverts this matrix, so a
zero cost reaches the CUDA kernel as an infinite stencil weight and leaves part
of the value function at infinity: measured on Two-Rooms, 425 557 voxels more
than the same problem posed with a single state. Free switching is expressed by
folding the controls into one state, not by a zero matrix.

`Planner.transitions` emits a `RuntimeWarning` when it sees one, rather than
refusing.

Two settings are in use:

* `k1 = 2`, `k2 = 3`, recommended for general use: the comparisons with the
  sampling planners (`scripts/acquire_comparison.py`, notebooks 05, 07 and 08,
  section 5 of notebook 04, and the examples) are run with it. Reversing in
  navigation stays cheap, and the manoeuvring mode is engaged where the
  geometry requires it, such as the docking slot or the narrow passage.
* `k1 = 10`, `k2 = 50`, the centre of the parameter study
  (`scripts/acquire_sensitivity.py`) and of the illustrations in notebooks 00
  and 03. The one-parameter sweeps move one coefficient at a time around it;
  `--part k1_k2_map` covers a grid of 22 pairs, `k1` in {1, 2, 5, 10, 20} and
  `k2` in {2, 3, 5, 10, 20, 50, 100} with `k1 < k2`, which holds both
  settings.

The state-system comparison (notebook 04,
`generators/state_system_comparison.py`) prices every switch at one unit,
`transitions=1 - eye(4)`, as the resolution study does on Narrow-Passage
below.

The resolution study has settings of its own. Two-Rooms and Warehouse are
solved at `k1 = 10`, `k2 = 50` without the cap on the obstacle cost
(`saturate=False`). The narrow-passage scene prices every mode switch at one
unit, `transitions=1 - eye(4)`, that is `k1 = k2 = 1`, outside the range
`transition_matrix` accepts; it passes the matrix to the planner directly, and
keeps the cap. `RESOLUTION_SOLVE` in `scripts/acquire_sensitivity.py` holds
these settings.

## The cost

`shfm.solver.Planner`.

| parameter | default | effect |
|---|---|---|
| `obstacle_cost` | `True` | the local cost grows near obstacles, pushing the path off the walls. `False` gives a pure geodesic, which runs close to them. |
| `fm2` | 1.0 | weight of the clearance term in the speed field |
| `saturate` | `True` | cap the clearance term, so the path is not drawn to the centre of open areas |

The speed field is `1 + fm2 · d(q)`, with `d` the distance to the nearest
obstacle in grid units, capped when `saturate` is on.

### Where the cap sits

`lift_to_configuration_space(..., saturation_cap=...)` takes the cap in metres.
The default is the full vehicle length, used by every study here except the
resolution study, which runs Two-Rooms and Warehouse without the cap (see
[the state system](#the-state-system)). A lower cap shortens the reach of the
cost: the path comes closer to the walls of open areas, and stays the same in
narrow passages, where the clearance never reaches the cap.
`scripts/acquire_sensitivity.py --part clearance_cap` measures the effect of
eight caps, from no obstacle cost to no cap (`generators/clearance_cap.py`).

## Discretisation

| parameter | default | note |
|---|---|---|
| `n_theta` | 100 | orientations sampled over `[0, 2π)` |
| `nitermax_o` | 4000 | outer iteration cap, raised from the library default |

The memory grows with `n_theta`: the value function is
`nx · ny · n_theta · n_states` floats. The distance transform gives the
orientation axis an effectively infinite sampling step, so that
clearances stay purely spatial: each one is measured in the plane, to the
nearest configuration in collision at the same heading.

`nitermax_o` is raised because the library default is low enough that the
front of a large grid can need more outer iterations. agd then stops with a
`ValueError` ("did not reach convergence"); [troubleshooting.md](troubleshooting.md#the-front-did-not-converge)
gives the fix.
