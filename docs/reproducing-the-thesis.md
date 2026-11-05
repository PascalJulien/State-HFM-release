# Reproducing the figures and tables of the thesis

This document links the repository to the PhD thesis of Julien Pascal,
*Planification de trajectoire sous contraintes pour robot à deux trains
directeurs indépendants. Application à la navigation d'un chariot à fourche
automatisé dans des entrepôts encombrés* (Université Clermont Auvergne,
Institut Pascal).

For each figure and table of the thesis that comes from a computation, it gives
the script or the notebook that produces it, the file it writes and the data it
reads. The numbers are those of the thesis; the LaTeX labels are given as
well. A figure script writes its files under the name the
thesis includes them by, so a file of `figures/` and the file of the thesis can
be compared directly.

Figures that are drawings (TikZ or pgfplots written in the text, photographs,
figures of other works reproduced in the state of the art) are not listed.

Every script below is run by `scripts/make_all_figures.py` and, one at a time,
by `shfm.generators.run(name)`. They read `data/` and need no GPU, except the
two marked GPU (fig. 4.8 b and 4.10), which solve; the acquisitions that
produced the data are given in the last column and need one, except
`--part envs` and `--part reference_swept`.

## The scripts

The studies are split in two steps. The acquisition scripts solve, on the GPU,
and write their results under `data/`; the figure scripts of `generators/`
read those results and draw, on the CPU. The results of every acquisition
ship with the repository, so every figure can be redrawn without a GPU, except
fig. 4.8 b and 4.10, whose scripts solve.

* `scripts/make_all_figures.py` redraws every figure into `figures/`, running
  the figure scripts, the notebooks and the examples; `--generators` runs the
  scripts only;
* `scripts/acquire_sensitivity.py` solves the parameter study (resolution,
  one-parameter sweeps, transition costs, cap on the obstacle cost) into
  `runs/sensitivity/`;
* `scripts/acquire_comparison.py` solves our side of the comparison with the
  sampling planners into `runs/comparison/`;
* `scripts/run_budget_campaigns.py` re-runs four of the campaigns of the
  sampling planners (all but Narrow-Passage) with a one-second budget and, as a
  control, with their own budget, into `runs/comparison/budget/`;
* `scripts/regenerate_campaigns.py` re-runs the five benchmark campaigns from
  `data/campaigns/settings.json` into `runs/`;
* `scripts/verify_campaigns.py` rebuilds one campaign, planner by planner, from
  the same settings, and compares it with the shipped one run by run.

The acquisition scripts write under `runs/`, never over the shipped files;
`SHFM_DATA_DIR=runs` points the figure scripts at a rebuilt set (the map
images are always read from `data/maps/`). The campaigns
of the sampling planners ship rather than being produced on first use: four of
the five planners are anytime, so a re-run grows a different tree and returns
different paths; RRT stops at its first solution and reproduces, its
normalised curvature aside ([external/NOTICE.md](../external/NOTICE.md)). The last
section, [What a rebuild changes](#what-a-rebuild-changes), says how close a
rebuild comes to the shipped results.

## Chapter 1

| thesis | script (`GENERATORS` name) | files | data, acquisition |
|---|---|---|---|
| Fig. 1.1 a, b `fig:creneau` | `comparison_plates.py` (`comparison`) | `creneau_2ws.pdf`, `creneau_4WS.pdf` | `data/comparison/campaigns`, `acquire_comparison.py --part campaigns` |

## Chapter 3

| thesis | script | files | data |
|---|---|---|---|
| Fig. 3.4 `fig:multi-balles` | `multiball.py` (`multiball`) | `multiball_combined.png` | none |
| Fig. 3.6 `fig:2D_and_3D_env` | `env_2d3d.py` (`env_2d3d`) | `env_2d3d_combined.png` | none |

Fig. 3.5 `fig:collision_check` is a TikZ drawing; `collision_check.py`
(`collision`) draws a test of the same kind for notebook 02, at the same two
headings, 0° and 10°, with a 2.6 m by 1.0 m rectangle turned about its centre
where the thesis turns the forklift about O_R.

## Chapter 4

| thesis | script | files | data |
|---|---|---|---|
| Fig. 4.4 `fig:tissot_f` | `tissot.py` (`tissot`) | `tissot_d_improved.pdf` | none |
| Fig. 4.6 `fig:ch4_selling` (weights) | notebook 09 | printed values | none |
| Fig. 4.7 a, b `fig:stencil` | `stencil_3d.py` (`stencil`) | `stencil_3d_a_darker_grid.png`, `stencil_3d_b_darker_grid.png` | none |
| Fig. 4.8 a `fig:fonction_navigation_carte` | `two_rooms_map.py` (`nav_function_map`) | `4WS_nav_function_map.pdf` | `data/sensitivity/env_two_rooms.npz` |
| Fig. 4.8 b `fig:fonction_navigation_U` | `nav_function.py` (`nav_function`), GPU | `4WS_nav_function.png` | `data/sensitivity/env_two_rooms.npz` |
| Fig. 4.10 a, b `fig:sans_systeme_etats` | `state_system_comparison.py` (`state_system`), GPU | `no_state.pdf`, `with_state.pdf` | `data/state_system/` |
| Fig. 4.11 `fig:ch4_eventails` (coordinates) | `control_fans.py` (`control_fans`) | `control_fans.tex`, `control_fans.pdf` | none |

Two of them are not the printed images:

* **4.10.** The printed figure was computed with an earlier version of the code,
  without the `cos(δ_R)` factor of the rotation rate and with a zero transition
  cost between two states. `state_system_comparison.py` solves the same scene
  with the package: the path with the state system is close to the printed
  one; without it, the path crabs on 44 % of its length (heading off by more
  than 20°) against 76 % in the printed figure, the effect of the `cos(δ_R)`
  factor on the counter-phase turns. The zero transition cost, which agd
  turns into an infinite stencil weight, is replaced by a single state holding
  both control sets.
* **4.11.** The figure is TikZ; `control_fans.tex` holds its 238 points,
  identical to those of the text.

## Appendices B and C

| thesis | script | files or output |
|---|---|---|
| Fig. B.1 `fig:stencil_construit` | `stencil_example.py` (`stencil_example`) | `stencil.png` |
| Table of § B.4.1 (Selling decomposition in 3D) | notebook 09 | printed values |
| Tab. C.1 `tab:stencil_example`, values of appendix C, U(q) = 9.001 | notebook 09 | printed values |

In table C.1, three rows of zero weight list offsets other than those the
notebook computes; they come from another obtuse superbase of the same matrix,
which the notebook checks, and do not change U(q).

## Chapter 5

Figures:

| thesis | script | files | data, acquisition |
|---|---|---|---|
| Fig. 5.1 to 5.3 (`fig:test_env`, `fig:test_env_2`, `fig:warehouse_env`) | none: the maps | `data/maps/two_rooms.png`, `narrow_passage.png`, `warehouse.png` | |
| Fig. 5.4 `fig:procedural_envs` | `procedural_envs.py` (`procedural_envs`) | `procedural_envs.pdf` | `data/campaigns/procedural.json.gz` |
| Fig. 5.5 `fig:resolution_overlay` | `resolution_study.py` (`resolution`) | `resolution_traj_overlay.png` | `data/sensitivity/resolution`, `acquire_sensitivity.py --part resolution` |
| Fig. 5.6 `fig:np_rasterisation` | `rasterisation_scheme.py` (`rasterisation`) | `np_rasterisation.pdf` | none (schematic) |
| Fig. 5.8 `fig:resolution_cost` | `resolution_study.py` (`resolution`) | `resolution_cost_convergence.pdf` | `data/sensitivity/resolution`, `--part resolution` and `--part memory` |
| Fig. 5.11 `fig:scenarios_R` | `parameter_sweeps.py` (`parameter_sweeps`) | `sens_situations.pdf` | `data/sensitivity/one_parameter` |
| Fig. 5.12 `fig:saturation` | `clearance_cap.py` (`clearance_cap`) | `saturation_metriques.pdf` | `data/sensitivity/clearance_cap`, `--part clearance_cap` |
| Fig. 5.13 `fig:saturation_traj` | `clearance_cap.py` | `saturation_traj_two_rooms.pdf` | same |
| Fig. 5.14 a, b (`fig:couts_k1_*`) | `parameter_sweeps.py` | `sens_k1_demitour.pdf`, `sens_k1_entree.pdf` | `data/sensitivity/one_parameter`, `--part one_parameter` |
| Fig. 5.14 c, d (`fig:couts_k2_*`) | `parameter_sweeps.py` | `sens_k2_accostage.pdf`, `sens_k2_np.pdf` | `data/sensitivity/k1_k2_map`, `--part k1_k2_map` |
| Fig. 5.15 a to d (`fig:gamma_rev_*`) | `parameter_sweeps.py` | `sens_gamma_rev_{demitour,sortie,entree,traversee}.pdf` | `data/sensitivity/one_parameter` |
| Fig. 5.16 a to d (`fig:vitesses_*`) | `parameter_sweeps.py` | `sens_nu_FWD_traversee.pdf`, `sens_nu_2WS_sortie.pdf`, `sens_nu_4WS_traversee.pdf`, `sens_nu_4WS_demitour.pdf` | same |
| Fig. 5.17 `fig:comparaison_trajectoires_two_rooms` | `comparison_plates.py` (`comparison`) | `env1_traj.png` | `data/comparison/campaigns`, `acquire_comparison.py --part campaigns`; `data/campaigns/` |
| Fig. 5.18 `fig:env1_metrics` | `comparison_plates.py` | `env1_metrics.pdf` | as 5.17 |
| Fig. 5.19 `fig:modes_two_rooms` | `mode_use.py` (`modes`) | `modes_two_rooms.pdf` | `data/comparison/modes`, `--part modes` |
| Fig. 5.20 `fig:comparaison_trajectoires_warehouse` | `comparison_plates.py` | `runs_warehouse.png` | as 5.17 |
| Fig. 5.21 `fig:warehouse_metrics` | `comparison_plates.py` | `warehouse_metrics.pdf` | as 5.17 |
| Fig. 5.22 `fig:comparaison_trajectoires_procedural` | `comparison_plates.py` | `run_0123.pdf` | as 5.17 |
| Fig. 5.23 `fig:procedural_metrics` | `comparison_plates.py` | `procedural_metrics.pdf` | as 5.17 |
| Fig. 5.24 `fig:aire_balayee` | `swept_area.py` (`swept_area`) | `violin_swept_{env1,warehouse,procedural}.pdf` | as 5.17; `data/sensitivity/benchmr_swept.csv`, `acquire_sensitivity.py --part reference_swept` |
| Fig. 5.25 a, b `fig:accostage` | `comparison_plates.py` | `creneau_2ws_nl.pdf`, `creneau_4ws_nl.pdf` | as 5.17 |
| Fig. 5.25 c | `comparison_plates.py` | `creneau_4ws_k1050_nl.pdf` | `data/comparison/reference_costs`, `--part reference_costs` |
| Fig. 5.26 `fig:accostage_sota` | `comparison_plates.py` | `accostage_sota.pdf` | as 5.17 |
| Fig. 5.27 a, c `fig:comparaison_passage_etroit` | `comparison_plates.py` | `4ws_only_shortcuts.pdf`, `2ws_only_shortcuts.pdf` | as 5.17 |
| Fig. 5.27 b | `comparison_plates.py` | `4ws_only_shortcuts_k1050.pdf` | as 5.25 c |

The thesis includes the three maps as `test_env.png`, `test_env_2.png` and
`warehouse_env.png`: the last two are identical to `narrow_passage.png` and
`warehouse.png`, the first has the same pixels as `two_rooms.png`.

Fig. 5.7 `fig:np_noeuds` and fig. 5.10 `fig:crabe_place` are TikZ drawings
written in the text; no script of the repository draws them.

The metric plates 5.18, 5.21 and 5.23 end with a panel of the approach heading
error (AHE), equation (5.3) `eq:ahe` of the thesis, which `comparison_plates.py`
computes beside the Bench-MR metrics. The printed plates were assembled from
the same data: their content is that of the files the script writes, and their
layout is the same, to within 3 pt on the Two-Rooms plate and 1 pt on the
others.

The printed `procedural_envs.pdf` was drawn by an older matplotlib (3.0,
against 3.7 here): same maps and same page size, the rendering of the cell
edges differs.

Tables, and the values of the text (files under `figures/tables/`):

| thesis | script | file |
|---|---|---|
| Tab. 5.1 `tab:valeurs_balayees` | `scripts/acquire_sensitivity.py` | `EXACT_VALUES`, `MAP_GRID` |
| Tab. 5.2 `tab:scenarios_sensibilite` | `scripts/acquire_sensitivity.py` | `SCENARIOS` (poses on the hundred-cell grid, doubled at 200): Traversée is `S1`, Demi-tour `R1`, Sortie `R2`, Entrée `R3`; `S5` is not in the table, whose docking and Narrow-Passage rows are the problems of those two campaigns (`--part k1_k2_map`) |
| Tab. 5.4 `tab:synthese_comparaisons` | `comparison_plates.py` | `comparison_summary.tex` (body of the table, AHE row included) |
| Tab. 5.6 `tab:budget_1s` | `budget_table.py` (`budget`) | `budget_table.tex` (body of the table), `budget.md`; data `data/comparison/budget`, `scripts/run_budget_campaigns.py`; both columns, the one-second budget and the original budget, come from these re-runs, not from `data/campaigns/` |
| Tab. 5.7 `tab:accostage_metriques` | `comparison_plates.py` | `docking.md` (AHE column included) |
| § 5.2.4 (resolution: lengths, times, memory) | `parameter_tables.py` (`parameter_tables`) | `parameter_study.md` |
| § 5.3 (sweeps, price of (2, 3)) | `parameter_tables.py` | `parameter_study.md` |
| § 5.3.2, 5.5 (cap on the obstacle cost, RRT* and Informed-RRT*) | `clearance_cap.py` | `clearance_cap.md` |
| § 5.5 (our paths on every scene, cusps of Two-Rooms) | `comparison_plates.py` | `comparison.md` |
| § 5.5, 5.6.1 (final approach: the AHE, defined in § 5.4.2) | `comparison_plates.py` | `approach.md` |
| § 5.5.1 (steering saturation, ratio of the coefficients nu_2WS / nu_FWD, rotation over 1 m) | `mode_use.py` | `modes.md` |
| § 5.5.3 (swept areas) | `swept_area.py` | `swept_area.md` |
| § 5.6 (docking and Narrow-Passage at (10, 50)) | `comparison_plates.py` | `reference_costs.md` |

Tab. 5.3 `tab:bilan_parametres` sums up the sweeps in words. Tab. 5.5
`tab:temps` gives the solve times of our planner on four grid sizes, from
125 000 to 8 million points. They depend on the machine: no measurement of
them ships here, and notebook 08 (section 3) measures them on the machine that
runs it.

## What a rebuild changes

The figures above are drawn from the data that produced the thesis, and come
out identical to the files of the thesis, apart from the creation date written
inside the PDF files and the exceptions given above: 4.10,
`procedural_envs.pdf`, and the metric plates 5.18, 5.21 and 5.23.

The acquisition scripts rebuild that data on the GPU. The solve times and the
memory are those of the machine. The paths of the parameter study, solved with
the multiprecision option of agd (`ROBUST` in `scripts/acquisition.py`), come
back identical to the last bit: resolution, one-parameter sweeps, grid of
transition costs, (2, 3) against (10, 50). The other solves (the comparisons,
the modes, the cap on the obstacle cost) run in single precision, and the
order in which the GPU updates the blocks of the grid then changes the values:
on most problems, two runs of the same solve differ by less than 1e-3 cell,
and one procedural map at the cap L_tot/4 shifts by 0.17 cell.
