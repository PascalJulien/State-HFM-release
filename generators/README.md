# Figure scripts

Each script draws one figure or one group of figures, and the tables of values
that go with them. They read `data/`, or `$SHFM_DATA_DIR` when it is set (for
instance to `runs/` after a rebuild by the acquisition scripts; a relative path
is taken from the current directory, and the map images always come from
`data/maps/`), and write into
`figures/`, or `$SHFM_FIGURE_DIR`; tables go to its `tables/` subdirectory.
The files keep the names the PhD thesis includes them by, so that the two can
be compared directly; [`docs/reproducing-the-thesis.md`](../docs/reproducing-the-thesis.md)
maps them. The labels drawn on the figures are in French, as in the thesis.

## Running them

The package runs a script in a process of its own, sets the output directory
and checks that every expected file was written:

```python
from shfm import generators
generators.run("tissot")          # one of the names in generators.GENERATORS
```

To run all of them, without the notebooks:

```bash
python scripts/make_all_figures.py --generators
```

Running a script directly also works, from this folder. Each script needs a
process of its own: the plotting tools of Bench-MR change settings of
matplotlib that the scripts drawn afterwards in the same process would inherit.

Two scripts solve on the GPU (`nav_function`, `state_system`); the others read
the results shipped under `data/` and run on the CPU. Those that measure paths
or draw through `shfm.plates`, `state_system` among them, need the Bench-MR
metric bindings (`scripts/setup.sh`).

## What is here

| name in `GENERATORS` | script | draws |
|---|---|---|
| `tissot` | `tissot.py` | Tissot's indicatrix of the 2WS and 4WS models |
| `multiball` | `multiball.py` | the disc covering, at 4, 8 and 16 discs |
| `env_2d3d` | `env_2d3d.py` | the lift from the plane to `(x, y, θ)` |
| `collision` | `collision_check.py` | the collision test, for notebook 02 |
| `nav_function` | `nav_function.py` | an isotropic value function over `(x, y, θ)`, no kinematics and no modes |
| `nav_function_map` | `two_rooms_map.py` | the map of the scene of `nav_function`, with its source |
| `stencil` | `stencil_3d.py` | the four 4WS controls, alone, then with the offsets of their stencils |
| `control_fans` | `control_fans.py` | the paths of the controls of each mode, as TikZ coordinates |
| `state_system` | `state_system_comparison.py` | paths without and with the state system |
| `procedural_envs` | `procedural_envs.py` | the four generated corridor scenes |
| `stencil_example` | `stencil_example.py` | the stencil of the worked example |
| `resolution` | `resolution_study.py` | cost, memory and convergence against grid size; the paths on each grid |
| `rasterisation` | `rasterisation_scheme.py` | why a coarse grid closes Narrow-Passage, a schematic |
| `parameter_tables` | `parameter_tables.py` | the tables of the parameter study |
| `parameter_sweeps` | `parameter_sweeps.py` | the one-parameter sweeps and the k2 sweep, as paths |
| `clearance_cap` | `clearance_cap.py` | the effect of the cap on the obstacle cost |
| `comparison` | `comparison_plates.py` | the comparison with the sampling planners: trajectories, metrics, tables |
| `modes` | `mode_use.py` | the 4WS paths against the steering saturation and the ratio of the speed coefficients nu_2WS / nu_FWD |
| `swept_area` | `swept_area.py` | the swept-area violins |
| `budget` | `budget_table.py` | the sampling planners with a one-second budget |

Support modules, not figures: `study_tools.py`, the helpers shared by the
scripts of the studies (loading the acquisitions, cleaning and measuring our
paths, drawing maps); `traj_render.py`, the 3D rendering of `nav_function`;
`viz_mpb.py`, the drawing helpers the plotting tools of Bench-MR import
(`shfm.plates` puts this folder on `sys.path` for them).
