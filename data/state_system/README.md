# Scene of the state-system comparison

The scene that `generators/state_system_comparison.py` plans on, to compare
paths without and with the state system.

| file | what it is |
|---|---|
| `bays.json` | one run of a Bench-MR result file: the environment (a 54 x 54 grid, the start and the goal) and the settings it was run with, including the footprint polygon the trajectory plot draws; no planner results |
| `bays.png` | the image Bench-MR read the grid from, dark pixels being obstacles (threshold 0.5, resampled to 54 x 54) |
| `footprint.svg` | the footprint envelope Bench-MR was given: a rectangle from -0.4845 to 3.649 along the vehicle axis, 0.7 wide |

One grid cell is one metre. The start is `(50, 30, pi/2)` and the goal
`(22, 45, 3 pi/2)`, in grid units and radians.

The grid is read back with `shfm.environments.from_benchmark_run`, which
rebuilds the array from the map string of the run, as the paths of the
generator are planned on it. The image and the footprint are the inputs of
that run, kept so that it can be repeated with Bench-MR; the generator does
not read them.

The file is the first run of the result file of the original comparison,
with the paths of the sampling-based planners and the precomputed distance
field removed, and the file names in its settings reduced to the names above.
