# API reference

The package has fourteen modules. The first five are the planner, the next
two clean the paths it returns; the rest are measurement, comparison and
drawing.
Everything listed here is public; anything not listed is an implementation
detail and may move.

Names re-exported at package level are marked **(top level)**, meaning
`from shfm import X` works.

`Planner`, `PlanResult` and `require_gpu` are reachable that way but are
absent from `shfm.__all__`, so `from shfm import *` gives the part of the
package that runs without a GPU. Name them explicitly. The solver module is
loaded when one of them is named; `cupy` and `agd` when a solve starts.

---

## `shfm.vehicle`: geometry and footprint

| | |
|---|---|
| `Vehicle(length, width, wheelbase, rear_axle_offset, n_balls)` | **(top level)** dimensions in metres, origin at the centre of the rear axle |
| `DEFAULT_VEHICLE` | **(top level)** the forklift this package is built around |

Methods: `metres_per_unit(zoom)`, `footprint_area()`, `corners(zoom)`,
`footprint_polygon(x, y, theta, zoom, flip_theta=True)`, `ball_centres(zoom)`,
`ball_radius(zoom)`, `to_svg(path)`.

`flip_theta` adds π to the heading. It defaults to true because `corners()`
puts the front of the vehicle on negative `x`; with it on, the footprint faces
`theta`. The heading of a pose does not depend on the order a path is
traversed, so the same holds for paths from the solver and from Bench-MR.

## `shfm.kinematics`: admissible velocities

| | |
|---|---|
| `control_set(theta, wheelbase_u, ...)` | **(top level)** `(4, 3, n_theta, 4)`: four controls, three components, orientations, four modes |
| `control_set_2ws(theta, wheelbase_u, **kw)` | **(top level)** the same, rear axle locked |
| `control_set_unstructured(theta, wheelbase_u, ...)` | **(top level)** `(8, 3, n_theta, 1)`: the eight extreme vectors in one state, no mode structure |
| `REFERENCE_COEFFICIENTS` | **(top level)** `nu_fwd=3.0, nu_2ws=1.5, nu_4ws=0.5, gamma_rev=0.5` |

`wheelbase_u` is the wheelbase in grid units, `wheelbase * zoom`. The
translations are in grid units and the angular rate in radians per grid unit, so
the two must share a scale.

## `shfm.states`: the four modes

| | |
|---|---|
| `NAV_FWD, MAN_FWD, MAN_BWD, NAV_BWD` | **(top level)** mode names |
| `SOLVER_ORDER` | **(top level)** the order the solver stacks them in |
| `THESIS_ORDER` | **(top level)** the order the thesis numbers them in |
| `transition_matrix(k1, k2, order=SOLVER_ORDER)` | **(top level)** symmetric matrix of switching costs |
| `to_thesis_index(mode)` | **(top level)** index of a mode in `THESIS_ORDER`, the numbering of the thesis, 0 to 3 (§ 4.3.1) |

Two index conventions coexist. Build the matrix from names with
`transition_matrix` rather than writing the array by hand.

## `shfm.environments`: occupancy and the lift

| | |
|---|---|
| `from_image(path, target_shape, zoom, n_theta=100, ...)` | **(top level)** dark pixels are obstacles |
| `from_occupancy(walls, zoom, n_theta=100, ...)` | **(top level)** from a boolean array |
| `from_benchmark_run(run, zoom, ...)` | **(top level)** from one run of a Bench-MR result file |
| `lift_to_configuration_space(walls_dist, theta, zoom, dxy, ...)` | **(top level)** sweep the disc-covered footprint over every orientation |
| `Environment` | **(top level)** the result: `walls`, `walls3`, `walls3_dist`, `walls3_dist_saturated`, `zoom`, `theta`, `vehicle` |

`from_image` transposes and flips on load, and resamples each axis
independently.

## `shfm.solver`: the eikonal solve

This module needs a CUDA GPU. It is imported lazily, so the rest of the
package stays usable without one.

| | |
|---|---|
| `Planner(env, k1=10.0, k2=50.0, obstacle_cost=True, saturate=True, fm2=1.0, controls=None, transitions=None, **coeffs)` | **(top level)** |
| `Planner.solve(seed, tips, verbosity=0, geodesic_mixed_state=True, free_start_mode=False, free_goal_mode=False, **kw)` | `seed` and `tips` are `(x, y, theta, state)` |
| `PlanResult` | **(top level)** `values`, `geodesics`, `seed`, `tips`, `env`, `hfm_in`, `hfm_out` (the input and output of agd), `path(i)`, `raw_path(i)`, `modes_used(i)`, `reached(i)` |
| `require_gpu()` | **(top level)** import the GPU stack with an explicit message when missing |
| `agd_location()` | path of the `agd` package in use |

`controls` and `transitions` replace the defaults to study variants: a control
set with a different number of modes, given as an array or as a callable
`f(theta, wheelbase_u, xp)`, and the matching matrix. The builders of
`shfm.kinematics` take `xp` as a keyword, so wrap them:
`controls=lambda th, wb, xp: control_set_2ws(th, wb, xp=xp, delta_f_max=1.05)`.

The default transition costs are the centre of the parameter study; the
comparisons and the examples pass `k1=2`, `k2=3`. When `controls` is given,
`**coeffs` are not used: pass the coefficients to the builder. When
`transitions` is given, `k1` and `k2` are not used.

`modes_used(i)` is keyed by the index of the mode in the control set, which
for the default set is its position in `SOLVER_ORDER`.

**`path()` versus `raw_path()`.** `path()` returns the geodesic in the order it
is driven, start to goal. The solver backtracks from the goal, so its raw output
runs the other way, and `raw_path()` gives it as produced. Bench-MR forms its
curvature triples from one end, so traversal order changes the normalised
curvature by a few percent. The comparison plates
measure curvature in the order of `raw_path()`, goal to start, on the path
cleaned by `dechatter` and `close_start`, and the Approach Heading Error in the
order of `path()`.

`geodesic_mixed_state` must stay on for a multi-mode model. Without it the state
is treated as an ordinary coordinate and interpolated: the mode column of the
path holds values between the integers (272 of the 404 points of the hello
world problem), or the path stops at the first switch.

`free_start_mode` seeds the start in every mode at zero cost: a vehicle at rest
leaves in any mode at no transition cost, and the backtracking ends on the seed.
`free_goal_mode` tries each tip in every mode and keeps the cheapest geodesic.
Extra keywords go to agd; the comparisons pass `geodesic_PastSeed_delay=128`,
the parameter study 1024 on its Two-Rooms situations, and `close_start` joins
a path that still stops short of the start.

**When a goal is not reached.** `solve` raises no error for a goal it cannot
reach, so that a loop over many problems runs through. It emits a
`RuntimeWarning` for each such goal, with its index, its pose and the probable
cause: the goal outside the grid, the vehicle in collision at the goal pose,
or the goal cut off from the start, enclosed or behind a passage the vehicle
does not fit through. The path of that goal is a single point. When the start
itself is outside the grid or in collision, one warning covers every goal. A
start beyond the grid is not rejected by agd, which wraps its index around and
propagates from another cell; its paths are flagged all the same.
A path whose backtracking stopped further than one vehicle length from the
start, the definition of a degenerate path in the studies, gets the same
warning, with that distance and the stopping criterion. `reached(i)` gives the
same verdict. Within that distance a path counts as reached
whatever its stopping criterion, `hfm_out["geodesic_stopping_criteria"]`:
valid paths stop `AtSeed`, and also `Stationnary` or `PastSeed` a few cells
from the start. Every message starts with `goal`, so
`warnings.filterwarnings("error", message="goal", category=RuntimeWarning)`
turns them into errors.

## `shfm.postprocess`: the extraction chatter

| | |
|---|---|
| `dechatter(path)` | the path with every run of three or more reversals spaced by at most two points collapsed to one point; `(path, n_removed, length_removed)` |
| `reversals(path)` | indices of the steps where the direction of travel flips |
| `chatter_runs(path, min_flips=3, max_gap=2)` | the runs of close reversals |

Near the seed, the backtracking can step back and forth along the heading, a
quarter of a cell each way, for tens of points. No manoeuvre of the vehicle
produces that pattern, and it would read as dozens of cusps; isolated
reversals, the real cusps, are kept.

## `shfm.splice`: exact start

| | |
|---|---|
| `close_start(path, seed, env, wheelbase_u, delta_f_max, delta_r_max=0.0, ...)` | the path with its first stretch replaced by a connection that starts exactly on `seed`; `(path, info)` |
| `dubins_path(q0, q1, radius, step=0.25)` | the shortest Dubins path between two poses |

The extraction stops within the interpolation cube of the source, up to a cell
and a half and a few degrees away. `close_start` connects the requested pose to
a point of the path within four turning radii: by a Dubins path in a navigation
mode, by a straight move inside the cone of the manoeuvring controls otherwise,
without entering an obstacle and without being longer, up to a small tolerance,
than the stretch it replaces. When no point qualifies the path is returned
unchanged; `info["joined"]` says which.

## `shfm.metrics`: measurement, by Bench-MR

Every metric but one is computed by the Bench-MR implementation through its
Python bindings, not reimplemented here. Build them with `./scripts/setup.sh`.

`path_length`, `curvature`, `max_curvature`, `normalized_curvature`,
`clearing_distances`, `aol`, `approach_heading_error`,
`angular_deviation_at_goal`, `euclidean_deviation_at_goal`, `swept_area`,
`summarize(path, env, goal)`, `bindings()`, `quiet()`.

`clearing_distances`, and `summarize` through it, call a Bench-MR binding that
also writes the map and the distances along the path to `debug.json` in the
working directory ([external/NOTICE.md](../external/NOTICE.md)).

`approach_heading_error(path, goal_theta, window)` is the exception: Bench-MR
has no such metric, so it is computed in Python, from the poses, for every
planner alike. It is the Approach Heading Error (AHE) of the thesis, eq. (5.3):
the mean of `|theta_i - goal_theta|`, each gap wrapped into `[0, pi]`, over the
points of the last `window` of the path, weighted by the length of the step
that leaves each point; in radians. `window` is in grid units; the thesis takes
the vehicle length, `vehicle.length * zoom`, which is what `summarize` uses.
Steps shorter than 1e-9 are dropped first, so that repeated points neither
weigh nor shift the window. The path is read in the order it is driven.

The angular and euclidean deviations at the goal are Bench-MR's. The published
data keep them, but the results no longer use them: the AHE took their place.
They exist only in their `_at_goal` form. Bench-MR also ships
one-argument versions that read the goal from a global only a full benchmark run
initialises, and dereference a null pointer outside one; they are not wrapped
here.

## `shfm.benchmark`: the sampling-based planners

| | |
|---|---|
| `make_config(map_path, start, goal, vehicle, zoom, footprint_svg, ...)` | build a Bench-MR configuration |
| `run(config, workdir=None, timeout=3600)` | run the binary, return the parsed results |
| `paths_by_planner(results, run_index=0)` | `{planner: (n, 3) array}` |
| `endpoints_of(run)` | `(start, goal)`, needed for the procedural generators |
| `reached_goal(plan)` | whether the plan terminates at the requested pose |
| `load_campaign(name)`, `campaign_path(name)`, `campaign_file(name, cache_dir=None)` | the shipped campaigns |
| `SMOOTHERS`, `STEER_FUNCTION` | the smoothers and the steering function the campaigns were run with |
| `TEMPLATE`, `CAMPAIGN_DIR` | the Bench-MR settings template, the folder of the shipped campaigns |
| `CAMPAIGNS`, `PLANNERS`, `BENCH_ROOT`, `BINARY` | |

`make_config` also drives Bench-MR's own scene generators:
`generator="corridor"` builds randomised branching corridors from a seed,
`generator="random"` scatters obstacles at a density. Those scenes choose their
own endpoints, so read them back with `endpoints_of`.

`campaign_file` unpacks a gzipped campaign for tools that need a file name. It
caches into a folder of the system temporary directory that belongs to the
current user, `shfm-<uid>`, by default; do not point it at `figures/`.

## `shfm.campaigns`: re-running the campaigns

| | |
|---|---|
| `settings(scene)` | the configuration a campaign was launched with, with the file names of this repository |
| `n_runs(scene)` | how many runs it holds |
| `make_mpb(scene, planner, ...)` | one Bench-MR job, configured as its campaign was |
| `regenerate(scene, runs=None, ...)` | re-run and merge into one result file |
| `strip_paths(data)` | a result file with the absolute paths Bench-MR writes replaced by file names |
| `SETTINGS_FILE`, `MAPS`, `FOOTPRINTS` | |

A Bench-MR result file carries its configuration twice and neither copy can be
re-run as it stands. `settings()` reads the top block, which is the one that was
given; the block inside each run is the state *after* it, with the environment
seed already advanced.

## `shfm.plates`: the comparison plates

This module wraps Bench-MR's own `trajectory.visualize` and
`plot_stats.plot_planner_stats`, with our paths injected through
`additional_path` and `custom_stats`.

`custom_stats`, `campaign_ahe`, `trajectories`, `metric_violin`,
`metric_plate`, `save_stats`, `load_stats`, `OUR_PLANNERS`.

The AHE panel is the one Bench-MR does not define: its title is added to
Bench-MR's table of names at import, and the campaign files do not hold its
values. `campaign_ahe(json_file, zoom)` measures them on the stored
trajectories, for the runs that reached the goal; `metric_violin` and
`metric_plate` call it when the AHE is among their metrics, so they take the
`zoom` the campaign ran at. `metric_plate` draws, two per row, the length, the
AOL, the maximum curvature and the minimum clearing, then the AHE centred on
the last row, as in the thesis. `metric_violin` and `metric_plate` write
`save_file` and a copy suffixed `_nolegend` under `shfm.figures.FIGURE_DIR`,
keeping only the base name of `save_file`; `trajectories` writes `save_file`
as given, with the same `_nolegend` copy beside it.

Both Bench-MR helpers draw on the *current* figure, so a second call without
`plt.close("all")` draws over the first.

## `shfm.plotting`: drawing conventions

`draw_map`, `draw_path`, `draw_footprints`, `draw_pose`, `axis_labels`,
`overlay_by_value`, `STATE_COLORS`.

Three rules are applied everywhere: axes in grid units with the metre
equivalence stated on the label, the footprint drawn as a dashed rectangle, and
start blue / goal red with a nose marking the heading, grey and black over a
path coloured by mode.

## `shfm.figures`: where figures are written

`FIGURE_DIR`, `save(fig, name, dpi=150, formats=("png",))`. `FIGURE_DIR`
honours `$SHFM_FIGURE_DIR` and otherwise resolves to `figures/` beside the
package.

## `shfm.generators`: running the figure scripts

`GENERATORS`, `run(name, figure_dir=None, timeout=3600)`, `figure_path(name)`.

`run` executes the script of `generators/` that draws a figure, in a process
of its own, and raises if the script fails or if a file it is supposed to
write is missing.
