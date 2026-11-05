# Troubleshooting

Most failures of a solve raise no error. The planner returns a path, or a
warning, and the cause has to be found from what it returned. This page goes
from what you see to the cause, how to check it, and the fix.

The messages below are quoted from the code, or were obtained by provoking them
in isolation, as noted.

Start with

```bash
python scripts/check_install.py
```

which says, part by part, what this machine can run: the package, a solve on
the GPU, the Bench-MR metrics, the notebooks. A missing part is reported on a
line of its own, with the first line of the error, and the script exits with a
non-zero status. On a machine without a usable GPU, for instance:

```
GPU (every solve)
  MISSING cupy and a CUDA device                     cudaErrorNoDevice: no CUDA-capable device is detected
...
Missing: cupy and a CUDA device.
See docs/install.md.
```

## Index

| what you see | where |
|---|---|
| `conda env create` runs for more than half an hour | [Installation](#conda-takes-more-than-half-an-hour) |
| `shfm.solver needs a CUDA GPU with cupy installed` | [Installation](#cupy-cannot-be-imported) |
| `cudaErrorNoDevice`, `cudaErrorInsufficientDriver` | [Installation](#no-cuda-device-or-a-driver-too-old) |
| the first solve takes ten seconds or more | [Installation](#the-first-solve-is-slow) |
| `OutOfMemoryError` | [GPU](#out-of-gpu-memory) |
| degenerate paths, no error, another program on the GPU | [GPU](#two-solves-on-one-gpu-at-the-same-time) |
| `did not reach convergence after maximum allowed number` | [GPU](#the-front-did-not-converge) |
| `Bench-MR metrics bindings not found`, `Bench-MR binary not found`, `benchmark_template.json` | [Bench-MR](#bench-mr-is-not-built) |
| `Cannot build Bench-MR, missing:` | [Bench-MR](#scriptssetupsh-stops-before-building) |
| `Could not find a configuration file for package "ompl"` | [Bench-MR](#cmake-cannot-find-ompl-15) |
| `pdftoppm not found` | [Bench-MR](#two-png-plates-are-not-redrawn) |
| `RuntimeWarning: goal ... not reached` | [Planning](#a-goal-is-not-reached) |
| a path through a wall or a gap narrower than the vehicle, no warning | [Planning](#a-path-through-a-wall-without-a-warning) |
| a path to the wrong place, no warning | [Planning](#a-path-to-the-wrong-place) |
| the path ends in a heading or a mode other than the one asked | [Planning](#the-final-heading-or-mode-is-not-the-one-asked) |
| dozens of cusps at the start, stopping criterion `Stationnary` | [Planning](#dozens-of-cusps-at-the-start) |
| `transition matrix has a zero off-diagonal entry`, `expected 1 <= k1 < k2` | [Planning](#the-transition-matrix) |
| fractional values in the mode column of a path | [Planning](#fractional-modes) |
| crab motion along open corridors | [Planning](#crab-motion-along-open-corridors) |

## Installation

### conda takes more than half an hour

* **Cause.** conda older than 23.10 uses its classic solver by default, which
  is slow on this environment.
* **Check.** `conda --version`.
* **Fix.** `mamba env create -f environment.yml`, the same with `micromamba`,
  or `conda env create --solver=libmamba -f environment.yml` where that solver
  is installed ([install.md](install.md)).

### cupy cannot be imported

```
RuntimeError: shfm.solver needs a CUDA GPU with cupy installed. See environment.yml for a working set of versions.
```

* **Cause.** The interpreter that runs the solve cannot import `cupy`: the
  environment is not active, or a Jupyter kernel starts another Python than the
  one of the environment. Importing `shfm` works anywhere; the error comes when
  a solve starts.
* **Check.** `python -c "import cupy"` in the shell. In a notebook,
  `import sys; print(sys.executable)` should name the Python of the `shfm`
  environment.
* **Fix.** `conda activate shfm`. For Jupyter, start it from the activated
  environment, or register its kernel once,
  `python -m ipykernel install --user --name shfm`, and choose that kernel in
  the notebook. `scripts/make_all_figures.py` puts its own interpreter first on
  `PATH` before running the notebooks, for the same reason.

### No CUDA device, or a driver too old

```
cupy_backends.cuda.api.runtime.CUDARuntimeError: cudaErrorNoDevice: no CUDA-capable device is detected
```

```
cupy_backends.cuda.api.runtime.CUDARuntimeError: cudaErrorInsufficientDriver: CUDA driver version is insufficient for CUDA runtime version
```

The first was provoked by hiding the GPU (`CUDA_VISIBLE_DEVICES=""`); the
second is the text cupy gives to that CUDA error.

* **Cause.** No NVIDIA GPU visible to the process: no device, no driver, or
  `CUDA_VISIBLE_DEVICES` set to nothing. Or a driver older than the CUDA
  runtime installed in the environment.
* **Check.** `nvidia-smi` lists the GPU and, in its header, the highest CUDA
  version the driver supports. `check_install.py` prints the runtime and the
  driver it finds, as `runtime 10020, driver 12070` for CUDA 10.2 and 12.7.
  `echo $CUDA_VISIBLE_DEVICES`.
* **Fix.** The solver has no CPU path: it needs an NVIDIA GPU and its driver.
  Building environments, measuring paths and drawing run on the CPU
  ([README](README.md#before-you-start)). For the driver, update it, or create
  the environment on the machine that runs it: conda chooses the CUDA runtime
  of cupy from the driver it finds ([`environment.yml`](../environment.yml)).

### The first solve is slow

* **Cause.** agd compiles its CUDA kernels the first time they are used, and
  cupy keeps the result on disk, in `~/.cupy/kernel_cache` or in
  `$CUPY_CACHE_DIR`. On the machine the results were produced with, a small
  solve took 12.8 s with an empty cache and 0.9 s once it was filled.
* **Fix.** None needed. The cache persists from one session to the next.

## GPU

### Out of GPU memory

```
cupy.cuda.memory.OutOfMemoryError: Out of memory allocating 1,234,567,890 bytes (allocated so far: 2,000,000,000 bytes).
```

The text is cupy's, with the sizes of the failed allocation; it was not
provoked on purpose.

* **Cause.** The grid needs more memory than the GPU has free, or memory is
  held by earlier solves of the same process, or by another process.
* **Check.** `nvidia-smi` gives the memory in use, and by which process;
  `check_install.py` prints the free memory. The peak of a solve grows with the
  number of points `nx · ny · n_theta`: 314 MB for 100 x 100 x 100, 760 MB for
  150 x 150 x 100, 1254 MB for 200 x 200 x 100, measured on an NVIDIA T1200
  (`figures/tables/parameter_study.md`, *Grid resolution*), about 0.3 GB per
  million points.
* **Fix.** Fewer points: crop the map to the area the vehicle uses, fewer
  headings (`n_theta=` in `from_image` and `from_occupancy`, at the cost of
  the angular resolution), or a coarser grid, which can close or open narrow
  passages (notebook 06). A `PlanResult` keeps the inputs of its solve on the
  GPU; in a long session, `del result` then
  `cupy.get_default_memory_pool().free_all_blocks()` releases the memory, as
  `scripts/acquisition.py` does after every solve.

### Two solves on one GPU at the same time

Not provoked on purpose, to keep the GPU in a sound state.

* **Symptom.** Degenerate paths, with no error: paths that stop far from the
  start, or a single point. When the stop is further than a vehicle length
  from the start, `solve` warns:
  `goal i of n, (x, y, θ), not reached: the path stops D grid units from the start, more than the vehicle length (stopping criterion S); PlanResult.reached(i) is False.`
  It does not warn when the stop is closer.
* **Cause.** Two processes solving on the same GPU at once: two notebooks, a
  notebook and a script, two scripts.
* **Check.** `nvidia-smi` lists more than one compute process. The same
  solve, run alone, gives another path.
* **Fix.** One solve at a time per GPU. Scripts can be queued behind a lock
  file, `flock /tmp/shfm-gpu.lock python my_script.py`. Let a running solve
  finish rather than kill its process: on the machine the results were
  produced with, killing a process in the middle of a CUDA kernel once left
  the GPU unusable until a reboot.

### The front did not converge

```
ValueError: Solver fast_iterative_method for kernel eikonal did not reach convergence after maximum allowed number 50 of iterations
```

Provoked with a low cap, `nitermax_o=50`, on Two-Rooms; the text is agd's,
and the number it gives is the cap, 4000 in this package.

* **Cause.** The front needs more outer iterations than `nitermax_o` allows,
  4000 in `Planner.solve`, which can happen on a large grid.
* **Check.** `result.hfm_out["stats"]["eikonal"]["niter_o"]`, on a solve that
  converged, gives the number of iterations it used: 196 on Two-Rooms at
  100 x 100 x 100.
* **Fix.** Raise the cap: `solve(..., nitermax_o=20000)`, as the parameter
  sweeps do (`ROBUST` in `scripts/acquisition.py`).

## Bench-MR

Bench-MR computes the metrics. Planning a path does not need it; measuring one
does, and so do notebooks 03, 05, 06, 07 and 08, the acquisition scripts,
the figure scripts that measure paths or draw Bench-MR plates, and the
campaigns.

### Bench-MR is not built

```
RuntimeError: Bench-MR metrics bindings not found. Build them with ./scripts/setup.sh, which produces <clone>/external/bench-mr/python/metrics_bindings*.so.
```

```
RuntimeError: Bench-MR binary not found at <clone>/external/bench-mr/bin/benchmark. Build it with ./scripts/setup.sh
```

```
RuntimeError: Bench-MR settings template not found at <clone>/external/bench-mr/bin/benchmark_template.json. Build Bench-MR with ./scripts/setup.sh, which writes it.
```

The first comes from `shfm.metrics`, the second from `shfm.benchmark.run`, the
third from `shfm.benchmark.make_config`.

* **Cause.** `scripts/setup.sh` has not been run, or it built the bindings for
  another Python than the one in use.
* **Check.** `check_install.py`, section *Bench-MR*: `MISSING metric bindings,
  for this Python`, `MISSING benchmark binary`, `MISSING settings template`.
* **Fix.** `./scripts/setup.sh`, with the environment active, so that the
  bindings are compiled for its Python ([install.md](install.md)).

### `scripts/setup.sh` stops before building

```
Cannot build Bench-MR, missing:
  - SBPL: pkg-config cannot find sbpl.pc (set PKG_CONFIG_PATH)
See docs/install.md, section Requirements.
```

The script checks a few dependencies before configuring, and lists those it
cannot find, among: `a Python interpreter (activate the shfm environment)`,
`Python 3.9 or later, found ...`, `cmake`, `pkg-config`, `a C++17 compiler`,
the SBPL line above, `pybind11 in the Python environment at ...`. The
requirements are in [install.md](install.md#requirements).

* **Fix.** Install what is listed. SBPL comes from source or from ROS Noetic
  (`ros-noetic-sbpl`); sourcing the ROS `setup.bash` sets `PKG_CONFIG_PATH`
  for it. A Python or pybind11 line means the environment is not active: run
  `conda activate shfm` first, or set `PYTHON=` to its interpreter.

OMPL, Boost and Eigen are not in that list; when one is missing or too old, the
error comes from CMake.

### CMake cannot find OMPL 1.5

```
CMake Error at CMakeLists.txt:48 (find_package):
  Could not find a configuration file for package "ompl" that is compatible
  with requested version "1.5".

  The following configuration files were considered but not accepted:

    <prefix>/share/ompl/cmake/omplConfig.cmake, version: 1.4.2
```

When no OMPL is found at all, the message starts with
`By not providing "Findompl.cmake" in CMAKE_MODULE_PATH this project has asked
CMake to find a package configuration file provided by "ompl", but CMake did
not find one.` Both were provoked with CMake 3.27 on a minimal project making
the same `find_package(ompl 1.5 REQUIRED)` call as Bench-MR's `CMakeLists.txt`,
line 48.

* **Cause.** An OMPL older than 1.5, such as the 1.4 of Ubuntu 20.04, or one
  installed where CMake does not look.
* **Check.** `pkg-config --modversion ompl`.
* **Fix.** Build OMPL 1.5 from source (the results were produced with 1.5.2)
  and give its prefix to CMake: `CMAKE_PREFIX_PATH=<prefix> ./scripts/setup.sh`.

### Two PNG plates are not redrawn

* **Symptom.** `figures/env1_traj.png` and `figures/runs_warehouse.png` are
  older than their PDF after `scripts/make_all_figures.py`. Without them,
  `shfm.generators.run("comparison")` raises
  `RuntimeError: comparison_plates.py did not write env1_traj.png, runs_warehouse.png`.
* **Cause.** `pdftoppm` is not installed. The figure script then writes
  `pdftoppm not found: env1_traj.png not written` to its output, and
  `shfm.generators.run` warns: `comparison_plates.py rewrote the PDF but not
  env1_traj.png, runs_warehouse.png; is pdftoppm (poppler) installed?`.
* **Check.** `which pdftoppm`.
* **Fix.** Install poppler (`poppler-utils` on Debian and Ubuntu).

## Planning

### A goal is not reached

`solve` raises no error for a goal it cannot reach. It emits one
`RuntimeWarning` per such goal, starting with `goal`, and `result.reached(i)`
is false; [api.md](api.md) gives the rules. The message ends with the probable
cause:

| end of the message | causes |
|---|---|
| `the goal is outside the grid` | a pose beyond the map; a pose in metres on a grid of cells larger than a metre (zoom below 1) |
| `the vehicle is in collision at the goal pose` | a goal against a wall; a pose read off the image without conversion; a pose in metres where grid units are expected, which moves it towards the origin |
| `the vehicle is in collision at the start, (x, y, θ)` | the same at the start; with several goals, a single warning, `goals 0 to n-1 of n not reached: ...` |
| `the start, (x, y, θ), is outside the grid` | a start beyond the map, on any side; with several goals, a single warning |
| `the goal cannot be reached from the start: it is enclosed, or behind a passage the vehicle does not fit through` | an enclosed goal; a passage closed by the disc covering or by a grid too coarse |
| `the path stops D grid units from the start, more than the vehicle length (stopping criterion S)` | a grid too coarse (`InWall`); a manoeuvre close to the start (`PastSeed`, `Stationnary`); two solves on one GPU |

* **Check.** Before solving, draw the map and the poses (`draw_map`,
  `draw_pose`). After the solve, `np.isfinite(result.values).any(axis=(2, 3))`
  holds the cells the front reached, in some heading and mode, and
  `(~env.walls3[i, j]).sum()` the number of headings the vehicle can take at
  cell `(i, j)`.
* **Fix.** By cause: move the goal off the wall or out of the enclosed area,
  give the poses in grid units, refine the grid. To stop at the first such goal,
  `warnings.filterwarnings("error", message="goal", category=RuntimeWarning)`.

### A path through a wall, without a warning

The vehicle passes where it cannot fit, and the solve reports nothing. The
causes all make the vehicle smaller against the map than it is, or the map
emptier:

* `zoom` left at 1, or set to the size of a cell instead of its inverse:
  `print(env)` gives the size of a grid unit;
* a grid too coarse, whose resampling has thinned the obstacles;
* a floor plan misread by `from_image`, which takes as obstacles only the
  pixels of value exactly 0 and can erase walls thinner than a cell;
  a ROS map's unknown space, grey, is read as free.

The opposite also happens: a PNG with a transparent background, black in its
colour channels, is read as an obstacle everywhere.

### A path to the wrong place

A pose read off an image, as a column and a row, is mirrored top to bottom on
the grid, and a heading measured on the image has its sign reversed
([`data/maps/README.md`](../data/maps/README.md)). The solve reaches
the mirrored pose without a warning. Draw the poses on the map before solving.
`plt.imshow(env.walls)` draws the grid transposed; use `draw_map`, or
`plt.imshow(env.walls.T, origin="lower")`.

### The final heading or mode is not the one asked

* A heading in degrees: 180 is read as 180 radians, -126.8 degrees once
  wrapped. A heading of absolute value above 2π is almost always in degrees;
  use `np.radians`.
* A mode in the thesis numbering: the mode of a pose is an index into
  `SOLVER_ORDER` (`nav_fwd, man_fwd, man_bwd, nav_bwd`), not into
  `THESIS_ORDER`. Use `SOLVER_ORDER.index("nav_bwd")`, or
  `free_goal_mode=True` when the mode at the goal does not matter.
* The direction of the final approach cannot be imposed: a goal given in
  `nav_fwd` and reached in reverse gets a switch on its last point
  ([`your_own_problem.ipynb`](../examples/your_own_problem.ipynb), section 3).

### Dozens of cusps at the start

* **Symptom.** Tens of reversals of the direction of travel within a cell of
  the start, read as cusps by the metrics, and the stopping criterion
  `result.hfm_out["geodesic_stopping_criteria"]` at `Stationnary`. On
  Two-Rooms read with `from_image` at 200 cells, 2WS, `k1 = 10`, `k2 = 50`: 64
  reversals, 61 of which `dechatter` removes.
* **Cause.** The start is seeded in the mode its pose names, and the path does
  not leave in that mode.
* **Check.** `chatter_runs(path)` (`shfm.postprocess`) finds the run.
* **Fix.** `free_start_mode=True`, with `geodesic_PastSeed_delay=128` (1024 on
  the Two-Rooms situations of the parameter study); then `dechatter` and
  `close_start`, as the studies do.

### The transition matrix

```
RuntimeWarning: transition matrix has a zero off-diagonal entry; the solver squares and inverts it, so that transition reaches the kernel as an infinite stencil weight and part of the value function will be left at infinity. For free switching, put the controls in a single state instead.
```

A zero cost passed as `transitions=`. The path comes back, longer than it
should be: 84.0 grid units against 63.9 with every switch at 1, on Two-Rooms
with the start and the goal of the hello world ([notes.md](notes.md#no-zero-transition-cost)).
Price every switch, or put the controls in a single state.

```
ValueError: expected 1 <= k1 < k2, got k1=2 k2=2
```

`transition_matrix` refuses `k1 >= k2` and `k1 < 1`. A matrix outside that
range, such as switches all at 1, goes in as `transitions=1 - np.eye(4)`, as
the resolution study passes it ([parameters.md](parameters.md#the-state-system)).

```
ValueError: transition matrix is (2, 2) but the control set has 4 state(s)
ValueError: the default four-mode transition matrix does not fit a control set with 2 state(s); pass transitions= explicitly
```

A control set given as `controls=` and a transition matrix of another size,
or none: pass one of shape `(n_states, n_states)`.

### Fractional modes

* **Symptom.** The mode column of a path holds values between the integers:
  272 of the 404 points of the hello world problem.
* **Cause.** `geodesic_mixed_state=False`: the backtracking treats the mode as
  an ordinary coordinate and interpolates it; the path can also stop at the
  first switch ([api.md](api.md)).
* **Fix.** Leave `geodesic_mixed_state` at its default, `True`.

### Crab motion along open corridors

* **Cause.** Speed coefficients that do not satisfy
  `nu_fwd > nu_2ws > nu_4ws`, or a `k2` low enough that manoeuvring costs
  little more than navigating.
* **Check.** `result.modes_used(i)`, keyed by the index of the mode in
  `SOLVER_ORDER`.
* **Fix.** `REFERENCE_COEFFICIENTS`, and `k1 = 2`, `k2 = 3` or higher;
  [parameters.md](parameters.md) explains what each one weighs, and notebook 04
  shows the planner with and without the state system.
