# What lands in `figures/`

`figures/` holds every figure and table of values the repository draws, as
`scripts/make_all_figures.py` writes them, under three kinds of name: those of
the thesis, those of the notebooks, those of the examples.

## Figure scripts: the names of the thesis

The scripts of `generators/` write their files under the names the PhD thesis
includes them by, which [reproducing-the-thesis.md](reproducing-the-thesis.md)
lists figure by figure. Most come as a PDF and a PNG of the same name; the
trajectory and metric plates drawn by Bench-MR come as a PDF only, and two of
them, `env1_traj` and `runs_warehouse`, also as a PNG rendered at 200 dpi by
`pdftoppm`, the form the thesis includes.

The tables of values go to `figures/tables/`: Markdown tables of the numbers
the text quotes, and the bodies of two LaTeX tables
(`comparison_summary.tex`, `budget_table.tex`).

`shfm.generators.GENERATORS` maps a short name to the script and to every file
that script writes. `shfm.generators.run` raises if any of them is missing, so
a run that stops halfway is caught.

| files | script |
|---|---|
| `tissot_d_improved.*` | `tissot.py` |
| `multiball_*.png` | `multiball.py` |
| `env_2d3d_*.png` | `env_2d3d.py` |
| `collision_check.*` | `collision_check.py` |
| `4WS_nav_function.png`, `4WS_nav_function_map.pdf` | `nav_function.py`, `two_rooms_map.py` |
| `stencil_3d_a_darker_grid.*`, `stencil_3d_b_darker_grid.*` | `stencil_3d.py` |
| `control_fans.*` | `control_fans.py` |
| `no_state.pdf`, `with_state.pdf` | `state_system_comparison.py` |
| `procedural_envs.*` | `procedural_envs.py` |
| `stencil.*` | `stencil_example.py` |
| `resolution_cost_convergence.*`, `resolution_traj_overlay.*` | `resolution_study.py` |
| `np_rasterisation.*` | `rasterisation_scheme.py` |
| `sens_*` | `parameter_sweeps.py` |
| `saturation_*`, `tables/clearance_cap.md` | `clearance_cap.py` |
| `env1_*`, `runs_warehouse.*`, `run_0123.pdf`, `warehouse_metrics.pdf`, `procedural_metrics.pdf`, `creneau_*`, `accostage_sota.pdf`, `2ws_only_shortcuts.pdf`, `4ws_only_shortcuts.pdf`, `4ws_only_shortcuts_k1050.pdf`, `tables/comparison.md`, `tables/comparison_summary.tex`, `tables/docking.md`, `tables/reference_costs.md`, `tables/approach.md` | `comparison_plates.py` |
| `modes_two_rooms.*`, `tables/modes.md` | `mode_use.py` |
| `violin_swept_*`, `tables/swept_area.md` | `swept_area.py` |
| `tables/parameter_study.md` | `parameter_tables.py` |
| `tables/budget*` | `budget_table.py` |

## Notebook figures: `NN_name`

A notebook writes a figure either with `shfm.figures.save`, which produces a
PNG, or as a Bench-MR plate through `shfm.plates`, which produces a PDF and
its `_nolegend` variant. The prefix is the number of the notebook that draws
it. Notebooks 02 and 06 write nothing of their own: they call
`shfm.generators.run` and display the files it returns.

| notebook | files |
|---|---|
| 00 | `00_overview.png` |
| 01 | `01_footprint.png`, `01_steering_modes.png`, `01_turning_radius.png` |
| 03 | `03_geodesic.png`, `03_obstacle_cost.png` |
| 04 | `04_with_without_states.png`, `04_mode_usage.png` |
| 05 | `05_creneau_2ws.pdf`, `05_creneau_4WS.pdf`, `05_accostage_sota.pdf`, `05_creneau_metrics.pdf`, `05_creneau_<metric>.pdf`, `05_2ws_only_shortcuts.pdf`, `05_4ws_only_shortcuts.pdf`, `05_shortcuts_2ws_4ws.pdf`, `05_creneau_4ws_k1050.pdf`, `05_4ws_only_shortcuts_k1050.pdf`, `05_crab.png` |
| 07 | `07_two_rooms_traj.pdf`, `07_warehouse_traj.pdf`, `07_procedural_traj.pdf` |
| 08 | `08_<scene>_metrics.pdf`, `08_<scene>_<metric>.pdf`, for `two_rooms`, `warehouse`, `procedural` |
| 09 | `09_consistency.png`, `09_update_roots.png` |

Notebook 07 also writes the statistics of our paths under `runs/`, for notebook
08 to read.

## Examples: the name of the example

The examples write their figures with `shfm.figures.save`, under the name of
the example.

| example | files |
|---|---|
| `examples/hello_world.py` | `hello_world.png` |
| `examples/your_own_problem.ipynb` | `your_own_problem.png`, `your_own_problem_settings.png` |

The notebook writes its other files, a floor plan and the CSV of its paths,
to a temporary folder, not to `figures/` or `data/`.

## What should not be there

`figures/` is for figures. Campaign caches do not belong in it: use
`shfm.benchmark.campaign_file(name)` with its default, which unpacks into the
system temporary directory.

## Names

The scenes and the parameters carry two sets of names: the ones of this
package, and the working names the campaigns and the figure files were
produced under, which remain in file names and CSV columns. The last column
gives the names the thesis uses.

| here | also appears as | in the thesis |
|---|---|---|
| `two_rooms` | `env1` (swept-area violins and plates), `S1`, `S5`, `R1` to `R3` (the situations of the parameter study, on the same map read without transposition), `test_env.png` (inside the campaign files) | Two-Rooms |
| situations `S1`, `R1`, `R2`, `R3` | `traversee`, `demitour`, `sortie`, `entree` (in the names of the `sens_*` files), Traversée, Demi-tour, Sortie, Entrée (notebook 06 and the panel titles of the parameter study) | Traversée, Demi-tour, Sortie, Entrée (tab. 5.2) |
| `narrow_passage` | `np`, `shortcuts`, `test_env_2.png` (inside the campaign file) | Narrow-Passage |
| `docking` | `creneau`, `accostage` | Accostage |
| `warehouse` | `warehouse_env.png` (inside the campaign file) | Warehouse |
| `procedural` | `run_0123` (fig. 5.22) | cartes procédurales |
| `SHFM_2WS`, `SHFM_4WS` (`OUR_PLANNERS`, the metric plots) | `shfm2ws`, `shfm4ws` (the trajectory plates, `ignore_planners`) | |
| `gamma_rev` | `nu_reverse` | γ_rev |
| `nu_fwd`, `nu_2ws`, `nu_4ws` | `nu_FWD`, `nu_2WS`, `nu_4WS` | ν_FWD, ν_2WS, ν_4WS |
| cap on the obstacle cost (`saturation_cap`, `saturate`) | `saturation` | plafond |
| vehicle length | `L` in the caps `L/4`, `L/2`, `L`, `2L` (`clearance_cap`, notebook 06) | L_tot; the thesis writes L for the wheelbase |
