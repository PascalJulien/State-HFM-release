# Comparison with the sampling planners

Our side of the comparison, written by `scripts/acquire_comparison.py`, and the
campaigns of the sampling planners run again at another time budget, written by
`scripts/run_budget_campaigns.py`. Read by the figure scripts
`comparison_plates.py`, `mode_use.py`, `swept_area.py` and `budget_table.py`.

| path | what it holds |
|---|---|
| `campaigns/` | every problem of the five campaigns of `data/campaigns/` (one per fixed scene, one per distinct procedural map: 24), both vehicles, k1 = 2, k2 = 3, reference speed coefficients (`REFERENCE_COEFFICIENTS`, the coefficients ν of the thesis), start mode free |
| `modes/` | the Two-Rooms problem, both vehicles, for five steering saturations, 0.3 to 0.7 rad (both axles for the 4WS vehicle, the front one alone for the 2WS one), and four ratios nu_2WS / nu_FWD |
| `reference_costs/` | the docking and Narrow-Passage problems at k1 = 10, k2 = 50 |
| `budget/<scene>_1s.json.gz` | the campaign of the scene, every setting unchanged except a planning budget of one second |
| `budget/<scene>_original.json.gz` | the same, run again at the budget of the campaign (15 s, 5 s for Warehouse) |

`campaigns/`, `modes/` and `reference_costs/` hold `rows.csv` (one row per
solve, metrics included, keyed by `key`, for instance `docking__run00__4WS`)
and `paths.npz` (the paths under the same keys, as extracted, in the order
they are driven, `(n, 4)`: x, y, theta, mode index in
`shfm.states.SOLVER_ORDER`). The figure scripts remove the extraction chatter
and close the start on the requested pose before measuring them; the metrics
of `rows.csv` are those of the paths as extracted.

The files of `budget/` are Bench-MR result files, with the trajectory of every
run, as those of `data/campaigns/`. The planners were given a wall-clock
budget: a re-run on another machine, or beside another load, gives other
paths.
