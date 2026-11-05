# Implementation notes

## Metrics

Every metric but the AHE is computed by Bench-MR, through `shfm.metrics`,
including for the paths this planner produces, so that both sides of the
comparison are measured by the same code. The Approach Heading Error (AHE) of
the thesis, `shfm.metrics.approach_heading_error`, has no Bench-MR counterpart:
it is computed in Python from the poses, for every planner alike, on the
trajectories stored in the campaigns for the sampling planners. Three metrics
were added to Bench-MR:

* the swept floor area, in `src/metrics/SweptAreaMetric.h`;
* the angular and euclidean deviations at the goal, which take the goal pose as
  an argument, since the global they would otherwise read is only initialised
  by a full benchmark run. The published data keep them, but the results no
  longer use them: the AHE took their place.

The normalised curvature was also corrected. [`external/NOTICE.md`](../external/NOTICE.md)
lists every modification of the vendored copy.

One exception: the one-parameter sweeps of `scripts/acquire_sensitivity.py`
measure the swept area with a lighter rasteriser of their own, for the
hundreds of solves they make.

`shfm.benchmark.make_config` also drives Bench-MR's own scene generators, not
only its image loader. `generator="corridor"` builds randomised branching
corridors from a seed, which is how the twenty procedural instances are
produced (the procedural campaign used `grid_size=(200, 200)`,
`corridor_radius=8`, `branches=100`, not the defaults;
`data/campaigns/settings.json` holds them); `generator="random"` scatters
obstacles at a given density. Those scenes choose their own endpoints, so read
them back with `shfm.benchmark.endpoints_of`.

## Path direction

`PlanResult.path()` returns the path in the order it is driven, start to goal.
The solver backtracks from the goal, so its raw output runs the other way;
`PlanResult.geodesics` still holds it as produced. The reversal matters: a
metric that looks at the final poses would otherwise score the departure.

## Mode numbering

The solver stacks the modes in the order `nav_fwd, man_fwd, man_bwd, nav_bwd`,
while the thesis numbers them `nav_fwd, nav_bwd, man_fwd, man_bwd`. Build the
transition matrix with `transition_matrix(k1, k2)` rather than writing the
array by hand, and pass `THESIS_ORDER` if you want the numbering used in the
text.

## The Bench-MR footprint

Upstream Bench-MR centres the robot polygon on its centroid and scales it by a
global factor. The vendored copy then shifts it so that the reference point
sits on the rear axle of this particular forklift, with an offset deduced from
the half width of the shape; that is what the shipped campaigns ran with. It
also adds three explicit settings, which take precedence when given:
`robot_shape_source`, `robot_shape_scale`, and the reference point
`robot_reference_x` / `robot_reference_y`.

The shipped campaigns predate those settings. Their sampling planners planned
with the envelopes of `data/footprints/`, which measure 4.133 m by 0.700 m on
the navigation scenes and 3.306 m by 0.560 m on the docking one, against
3.649 m by 0.700 m for the vehicle this package declares;
`data/footprints/README.md` has the details.

## No zero transition cost

`agd` passes the transition matrix to its kernel as
`state_transition_costs ** -2`, so an entry of zero arrives as an infinite
stencil weight and leaves part of the value function at infinity, with no error
raised and a plausible path returned. Free switching between controls is
expressed by putting them in the same state, the way
`control_set_unstructured` does, not by a zero matrix. `Planner.transitions`
warns when it sees one.
