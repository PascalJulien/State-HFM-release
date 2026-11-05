# Documentation

* **[parameters.md](parameters.md)**: the model and its parameters (units and
  `zoom`, the vehicle, the speed coefficients, the transition costs, the
  obstacle cost, the discretisation). Read this one first.
* **[api.md](api.md)**: what each module exports, and what to know before
  calling it.
* **[figures.md](figures.md)**: the files written to `figures/`, and what
  draws each of them.
* **[reproducing-the-thesis.md](reproducing-the-thesis.md)**: the link with the
  PhD thesis the method comes from: for each of its figures and tables that
  come from a computation, the script, the file and the data.
* **[notes.md](notes.md)**: how the metrics are computed, the order of the
  paths and of the modes, the Bench-MR footprint, why no transition cost is
  zero.
* **[install.md](install.md)**: requirements, the configuration the results
  were produced with, and the build of Bench-MR.
* **[troubleshooting.md](troubleshooting.md)**: from a symptom to its cause,
  how to check it and the fix: installation, GPU, Bench-MR, planning.

The notebooks form the tutorial, to be read in order starting with
`notebooks/00_overview.ipynb`. `examples/hello_world.py` is the shortest
program that plans a path, and `examples/your_own_problem.ipynb` applies the
planner to a problem of your own. This folder holds the reference
documentation.

## Before you start

**The solver needs a CUDA GPU.** There is no practical CPU path. Everything
else, building environments, computing metrics, drawing, runs on the CPU, and
importing `shfm` on a machine without a GPU works: the GPU stack is loaded
when a solve starts.

**Lengths are in grid units, not metres.** One unit is `1 / zoom` metres. The
only quantity returned in SI is the swept floor area, in square metres. See
[parameters.md](parameters.md#units).

**Paths are extracted in reverse.** The solver backtracks from the goal.
`PlanResult.path()` returns the path in the order it is driven;
`PlanResult.raw_path()` returns it in the solver's own order; the comparison
plates measure curvature in that order, on the path cleaned by
`dechatter` and `close_start`. The metrics that look at the final poses depend
on which one is used.

## Repository layout

```
shfm/               the package
  vehicle.py        dimensions, footprint, disc covering
  kinematics.py     admissible velocities per mode
  states.py         the four modes and the transition matrix
  environments.py   occupancy maps and the lift to configuration space
  solver.py         the eikonal solve
  postprocess.py    removal of the extraction chatter
  splice.py         exact start of an extracted path
  metrics.py        length, curvature, clearance, swept area, approach heading
  benchmark.py      configuring and running the Bench-MR planners
  campaigns.py      building and re-running the Bench-MR campaigns
  plates.py         the comparison plates, drawn by Bench-MR's own tools
  plotting.py       drawing conventions
  figures.py        where figures are written
  generators.py     running the figure scripts below
generators/         the figure scripts
scripts/            setup, acquisitions, campaigns, redrawing every figure, checks
notebooks/          the tutorial, in order
examples/           a first script, a problem of your own
data/maps/          the four scene images
data/footprints/    the SVG envelopes the campaigns used
data/campaigns/     the Bench-MR runs of the sampling planners, and their settings
data/sensitivity/   the parameter study: resolution, sweeps, cap on the obstacle cost
data/comparison/    our paths on the campaign problems, and four campaigns run again at 1 s and at their own budget
data/state_system/  the scene of the comparison without and with the state system
data/forklift.svg   the vehicle, in metres, origin at the rear axle
figures/            every figure and table of values, as the scripts draw them
external/agd        the solver library, vendored
external/bench-mr   the benchmark suite, vendored and modified
external/NOTICE.md  what is vendored, from where, and what was changed
docs/               reference documentation
```

`external/` holds third-party code only. `runs/`, where the rebuilds are
written, is not versioned.
