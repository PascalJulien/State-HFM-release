# Vendored dependencies

Both libraries below are shipped in this repository rather than fetched at
install time, so that the code can be run years from now without depending on
an index or a remote host staying available. Neither is our work.

## `agd`: Adaptive Grid Discretizations

Copyright 2019 Jean-Marie Mirebeau, University Paris-Sud, CNRS, University Paris-Saclay.
Upstream: <https://github.com/Mirebeau/AdaptiveGridDiscretizations>
Version vendored here: 0.2.5, the version upstream declares for this code in
`conda.recipe/setup.py`. The package directory is copied from the upstream
repository unmodified, and the modules it shares with the 0.2.5 release on
PyPI are identical to those of that release. Two files were added to this
copy and are not upstream files: `agd/agd/setup.py` and `agd/agd/README.md`.

It provides the anisotropic eikonal solver and its CUDA kernels, including the
`DubinsState` model this work extends, which J.-M. Mirebeau wrote following
our exchanges.

Only the Python package `agd/agd/` is vendored. Its author distributes it under
the Apache License 2.0: see `agd/agd/License.txt`, the notice in the source
headers and the licence classifier of the 0.2.5 release on PyPI. The modules
under `agd/agd/ExportedCode/` are code exported from the upstream notebooks;
they are part of the package as its author distributes it, and its licence
covers them. The upstream repository is otherwise a collection of notebooks and
their documentation, distributed under the GNU General Public License, version
3 or later. Those notebooks and that documentation are not included here, and
neither is the GPL licence file that covers them.

Two parts of the package carry notices of other authors in their headers:
the linear programming solver of `agd/agd/Eikonal/HFM_CUDA/cuda/LinProg/`, by
Michael E. Hohmeyer (1990), whose notice allows modification and
redistribution provided it is kept; and the sorting networks of
`agd/agd/Eikonal/HFM_CUDA/cuda/NetworkSort.h`, taken from a page by John M.
Gamble, whose licence notice is quoted there. The model this package runs
uses neither.

## `bench-mr`

Copyright 2017 Eric Heiden, MIT License (see `bench-mr/LICENSE`).
Upstream: <https://github.com/robot-motion/bench-mr>

The licences of the material Bench-MR includes (OMPL, SBPL,
astar-algorithm-cpp, theta_star, steering_functions, POSQ) are in
`bench-mr/3rd_party_licenses.md`, as upstream ships it. The components bundled
under `bench-mr/third_party/` carry their own licences: `collision2d`, `params`
and `stb` in their `LICENSE` files; `steering_functions` (Apache License 2.0)
in `LICENSE`, `NOTICE` and `3rd-party-licenses.txt`, the last one listing its
own dependencies; `chomp` has no licence file of its own, and its sources carry
a BSD-style notice by Matt Zucker in their headers.

### What is left out

The copy keeps the upstream sources, the resources of `bin/` and the Python
tools of `python/`, with the licence and README files of each component. It
leaves out:

* the build outputs, result files and logs, the upstream `README.md`, the
  notebooks of `python/` with its `README.md`, `requirements.txt` and
  `plots/`, the Docker, Doxygen and continuous-integration files, and the
  Moving AI scenarios of `bin/moving_ai_scenarios/`;
* of `stb`, everything except the three headers Bench-MR includes,
  `stb_image.h`, `stb_image_resize.h` and `stb_image_write.h`; its `README.md`
  still lists the other libraries of the collection;
* of `steering_functions`, the tests, `doc/`, `scripts/`, `visualize_ompl.py`,
  the ROS node `src/steering_functions_node.cpp` and `CONTRIBUTING.md`; its
  `README.md` still links to the last one and to an image under `doc/`;
* of `chomp`, `paper/` and the demonstration maps of `maps/`;
* of `collision2d`, the demonstration program and its `CMakeLists.txt`, which
  the build does not include.

The thirteen programs of `experiments/` other than `benchmark.cpp` are kept, as
are `third_party/params/demo.cpp` and `third_party/steering_functions/src/ompl_demo.cpp`:
the CMake files declare them, and configuring fails if they are missing.
`scripts/setup.sh` builds only the `benchmark` and `metrics_bindings` targets.

### Modifications

This copy is modified. The changes below are measured against upstream commit
`c10ef1c` and the submodule commits it records.

1. **Four-wheel steering**, so that sampling-based planners can be run on the
   same kinematics as this planner.

   * Three steering types in `src/steer_functions/Steering.h`:
     `STEER_TYPE_4WS_POSQ` (8), `STEER_TYPE_4WS_DUBINS_EXTENDED` (9) and
     `STEER_TYPE_4WS_STATE_SPACE` (10). They are not equivalent, and only the
     last one is a four-wheel-steering model. In `src/base/src/PlannerSettings.cpp`,
     type 10 builds a `FourWSStateSpace` from the inverse of
     `car_turning_radius`, `sampling_resolution`, `wheel_base` and
     `delta_max`; it is the type that `shfm.benchmark.make_config` selects
     under the name `"4ws_state_space"`. Type 9 builds a plain
     `ob::DubinsStateSpace`, as the ordinary Dubins type does, so its
     `4WS` label describes an intent rather than the model. The branch of
     type 8 is commented out, so that type ends in the "Unknown steer
     function" error and stops the program. All three are added to the
     steering types the Halton sampler accepts.
   * The state space of type 10, in `third_party/steering_functions`: the
     added files `include/steering_functions/4ws_state_space/4ws_state_space.hpp`
     and `src/4ws_state_space/4ws_state_space.cpp` are derived from the
     Reeds-Shepp state space of that library. They evaluate the Reeds-Shepp
     families and add a straight crab segment, kept when the direction of the
     displacement lies within `delta_max` of the heading and its penalised
     length is below that of the best Reeds-Shepp path. Four files of that
     library are modified for it: `CMakeLists.txt` compiles the new source,
     `include/ompl_state_spaces/CurvatureStateSpace.hpp` declares the OMPL
     class `FourWSStateSpace`, and
     `include/steering_functions/utilities/utilities.hpp` and
     `src/utilities/utilities.cpp` add the function `end_of_crab_line`.
   * Settings, in `src/base/PlannerSettings.h`: `wheel_base` (default 11.04)
     and `delta_max` (default 0.5) in the `steer` and `forwardpropagation`
     groups. The default list of `benchmark.steer_functions` gains types 8
     and 9 after CC Reeds-Shepp, and the default list of
     `benchmark.forward_propagations` gains the four-wheel-steering model.
     Since type 8 stops the program, a configuration has to set
     `steer_functions`; `make_config` and the shipped campaigns do.
   * A forward-propagation type `FORWARD_PROPAGATION_TYPE_KINEMATIC_4WS` (2)
     in `src/fp_models/ForwardPropagation.h`, with its model in
     `src/fp_models/kinematic_4ws/kinematic_4ws.hpp`. It is wired in
     `PlannerSettings.cpp` (an SE(2) state space with three controls, speed
     and the front and rear steering angles), `AbstractPlanner.cpp` (state
     validity checker, start and goal) and `OMPLControlPlanner.hpp` (ODE
     propagator).
   * The upstream kinematic-car model, `src/fp_models/kinematic_car/kinematic_car.hpp`,
     is itself given a rear steering angle, and its control space, in
     `PlannerSettings.cpp`, a third input.

   These C++ models compute the rotation rate as `v (tan delta_F - tan
   delta_R) / L`, without the `cos(delta_R)` factor that `shfm.kinematics`
   applies for a speed measured at the rear-axle centre.

   None of this affects any published result. The five campaigns in
   `data/campaigns/` all carry `steer_functions: [0]`, that is Reeds-Shepp,
   with the control-based planners switched off, and `make_config` exposes
   only `"reeds_shepp"` and `"4ws_state_space"`, so types 8 and 9 are
   unreachable from this package.
2. **Placement of the robot polygon**, in
   `CollisionSettings::initializeCollisionModel`. Upstream centres the polygon
   on the centroid of its vertices and scales it by the global
   `env.polygon.scaling`. This copy then places it in one of two ways.

   * By default, it shifts the shape along its length so that the reference
     point sits on the rear axle of a 3.6490 m vehicle whose axle is 0.4845 m
     from its rear, with a scale deduced from the half width of the shape.
     That is the placement the campaigns shipped in `data/campaigns/` were
     run with.
   * When the setting `robot_shape_scale` is positive, it scales the shape by
     that value instead, and translates it so that the point
     (`robot_reference_x`, `robot_reference_y`) of the centred and scaled
     shape lands on the origin. The shipped campaigns do not carry this
     setting; `shfm.benchmark.make_config` sets it, with a reference point at
     (0, 0).

   The bounds of the shape are logged after this placement.
3. **Metrics.**

   * A new metric, the swept floor area, in `src/metrics/SweptAreaMetric.h`,
     computed only through the Python bindings.
   * Two new metrics, the angular and euclidean deviations at the goal, in
     `src/metrics/AngularDeviationMetric.h` and `EuclideanDeviationMetric.h`,
     recorded in `PathStatistics.hpp` as `angular_deviation_metrics` and
     `euclidean_deviation_metrics` and computed for every solution in
     `PathEvaluation.hpp`. Each has a form that reads the goal from the
     global settings, which only a full benchmark run initialises, and a form
     that takes it as an argument; `shfm.metrics` wraps only the second.
   * The normalised curvature, `NormalizedCurvatureMetric.h`, differs from
     upstream in three points: the points of a triple are at least 0.1 apart
     instead of 0.3, nearly collinear triples are skipped, and a radius below
     `1e-10` is given the maximum curvature instead of its inverse. The centre
     of the circle through a triple is computed as upstream computes it,
     written with one denominator whose opposite divides the ordinate. The
     campaigns shipped in `data/campaigns/` were run with an earlier build of
     this copy that divided both coordinates by the same value, which places
     the centre at `(cx, -cy)` and makes the metric depend on where the path
     lies in the map: a quarter circle of radius 5 sampled at 200 points
     scores 1.33 centred on the origin and 0.02 centred on (100, 160). The
     normalised curvature stored in their statistics carries that error.
   * The mean clearing distance is computed in place in `ClearingMetric.h`
     rather than through `stat::mean`, with the same result.
   * Console messages are silenced in `PathLengthMetric.h` and `GridMaze.cpp`.
4. **Python bindings** of the metrics, in `src/bindings/`, built as the
   `metrics_bindings` CMake target into `python/`. The top-level
   `CMakeLists.txt` adds that directory and compiles the project as
   position-independent code. The module exports `compute_path_length`,
   `compute_curvature`, `compute_aol`, `compute_clearing_distances`,
   `compute_swept_area`, the two deviations at the goal and their
   one-argument forms, and `compute_smoothness`, which is not implemented and
   returns 0.0. `compute_clearing_distances` builds a `GridMaze` from the
   occupancy grid it is given, stores it in `global::settings.environment`
   and writes the environment and the distances along the path to
   `debug.json` in the working directory. Part of the docstrings are in
   French.
5. **Environments**, in `GridMaze`: the map distances are always written with
   the environment, a `setOccupancyGrid` method is added, and the corridor
   generator moves the goal back along its heading by `7 - radius` when the
   corridor radius is below 7 cells. The procedural campaign uses a radius of
   8, so it is not affected.
6. **Python tools**, under `python/`.

   * `plot_stats.py`: `plot_planner_stats` and `plot_smoother_stats` take
     `custom_stats`, inserted as their second parameter, whose values are
     added to the violins. In `plot_planner_stats` the palette grows by two
     colours when `custom_stats` is given, and its last two colours are always
     blue and red. Violins are drawn with a visible edge and an opacity of
     0.85. Each combined figure is saved a second time without its legend, as
     `<name>_nolegend.<ext>`.
   * `trajectory.py`: `visualize` takes `additional_path`, two lists of paths
     indexed by run, drawn in blue and red as `shfm2ws` and `shfm4ws`. With
     `combine_views`, the legend is drawn once for the figure, on its right,
     under the title "Planificateur", the spacing between panels is changed,
     and the figure is also saved without its legend, as
     `<name>_nolegend.<ext>`. `visualize_grid` no longer draws a legend with
     `combine_views`.
   * In both files, when the module `viz_mpb` of `generators/` can be
     imported, the save path goes through `viz_mpb.redirect`.
   * `plot_env.py` shortens the title of image scenes to `image`.
   * `definitions.py` names the added metrics, steering types, robot model and
     planners (`shfm2ws`, `shfm4ws`).
   * `utils.py` imports the bindings when it is loaded, so the plotting
     modules that import it need them built, and gains `evaluate_path_metrics`,
     which no code calls.
   * The default binary directory of `mpb.py` is changed; `shfm` sets it at
     run time.
