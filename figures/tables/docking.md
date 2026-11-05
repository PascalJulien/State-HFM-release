# Docking: metrics of every planner

Sampling planners: mean ± sample standard deviation over the twenty runs of the campaign, all runs counted; normalized curvature measured again on the trajectories. AHE: over the runs that reached the goal (RRT 20, RRTstar 20, InformedRRTstar 20, SPARS 20, SST 13), mean ± population standard deviation. Ours: k1 = 2, k2 = 3, deterministic, one path.

| planner | length (cells) | norm. curvature | max. curvature (1/cell) | min. clearing (cells) | AHE (rad) |
|---|---|---|---|---|---|
| RRT | 166.350 ± 48.473 | 8.397 ± 5.918 | 2.365 ± 2.013 | 11.423 ± 1.961 | 0.793 ± 0.382 |
| RRTstar | 97.473 ± 2.075 | 3.238 ± 1.041 | 2.218 ± 1.985 | 10.323 ± 1.151 | 0.536 ± 0.088 |
| InformedRRTstar | 96.726 ± 2.151 | 2.874 ± 0.793 | 1.904 ± 1.676 | 9.988 ± 1.459 | 0.484 ± 0.076 |
| SPARS | 135.274 ± 26.605 | 6.018 ± 2.672 | 2.357 ± 1.763 | 12.462 ± 1.727 | 0.725 ± 0.266 |
| SST | 134.859 ± 22.287 | 8.496 ± 3.030 | 3.833 ± 1.661 | 12.146 ± 1.631 | 0.740 ± 0.235 |
| SHFM 2WS | 114.486 | 10.192 | 4.848 | 13.727 | 0.468 |
| SHFM 4WS | 86.195 | 3.259 | 0.652 | 15.000 | 0.330 |
