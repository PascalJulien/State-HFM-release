# Parameter study

Written by `scripts/acquire_sensitivity.py` (one directory per part), read by
the figure scripts `resolution_study.py`, `parameter_tables.py`,
`parameter_sweeps.py` and `clearance_cap.py`. Our planner, solved on the GPU.

| path | part | what it holds |
|---|---|---|
| `resolution/sweep_resolution.csv` | `resolution` | Two-Rooms, Narrow-Passage and Warehouse on six grids, 50 to 200 cells a side: solve time, GPU memory held by the pool, path metrics |
| `resolution/sweep_resolution_geos.npz` | `resolution` | the maps, the endpoints and the paths of every grid (`<scene>__<Nx>x<Ny>x100`, `(n, 4)`) |
| `resolution/memory.jsonl` | `memory` | peak GPU memory of each solve, measured in a fresh process |
| `one_parameter/env_two_rooms.npz` | `one_parameter` | Two-Rooms at 200 cells a side, as the sweeps solve it |
| `one_parameter/geos_1d_two_rooms.npz` | `one_parameter` | one path per situation, parameter and value (`<situation>__<parameter>__<value>`), as the solver returns it: `(4, n)`, goal to start |
| `one_parameter/swept_1d_two_rooms.csv` | `one_parameter` | one row per solve: swept area, reach error, length, cusps, shares of backward driving and of manoeuvring |
| `k1_k2_map/` | `k1_k2_map` | the grid `1 <= k1 < k2` on the situations and on the docking and Narrow-Passage problems |
| `recommended/` | `recommended` | (2, 3) and (10, 50) on the situations, solved as the sweeps are |
| `clearance_cap/` | `clearance_cap` | eight caps on the obstacle cost, Two-Rooms, Warehouse and the twenty procedural maps, both vehicles |
| `benchmr_swept.csv` | `reference_swept` | the swept area of every trajectory of the sampling planners in `data/campaigns/` |
| `env_two_rooms.npz` | `envs` | Two-Rooms at a hundred cells a side, read by `nav_function.py` |

`k1_k2_map/`, `recommended/` and `clearance_cap/` hold `rows.csv` (one row per
solve, metrics included, keyed by `key`) and `paths.npz` (the paths under the
same keys, in the order they are driven, `(n, 4)`: x, y, theta, mode index in
`shfm.states.SOLVER_ORDER`).

The situations are poses on Two-Rooms, defined on the hundred-cell grid
(`SCENARIOS` in the script) and doubled at 200: S1 crossing from one room to
the other, S5 along the corridor, R1 U-turn in the corridor, R2 leaving a room
whose back wall the vehicle faces, R3 ending in the other room facing its door.
This map is read as the image stands, rows as `x`, where Bench-MR and the
campaigns transpose it (see `data/maps/README.md`).

The times and the memory belong to the machine that measured them: an NVIDIA
T1200 with 4 GB, otherwise idle.
