# Maps

The four scenes, as images: dark pixels are obstacles. These are the inputs, not
the grids. Bench-MR resamples each one to the size its configuration asks for,
and `shfm.environments.from_image` does the same on this side; both land on
grids of the same size, which agree up to a few cells along the edges of the
obstacles (4 on Two-Rooms, 29 on Narrow-Passage, 2 on the docking slot) and up
to about 2000 of the 30 000 cells of Warehouse, whose image is resampled by a
non-integer factor. The studies on the campaign problems plan on the grid of
the campaign itself, read with `shfm.environments.from_benchmark_run`.

| file | image | grid the campaign plans on | one grid unit |
|---|---|---|---|
| `two_rooms.png` | 100x100 | 200x200 | 0.125 m |
| `narrow_passage.png` | 100x100 | 200x200 | 0.125 m |
| `warehouse.png` | 63x42 | 200x150 | 0.5 m |
| `docking.png` | 50x75 | 100x150 | 0.05 m |

The fifth scene has no image: the procedural corridors are generated from a
seed, and come back inside the campaign that used them.

One scene is read in two orientations. The parameter sweeps, their situations
(`S1`, `S5`, `R1` to `R3`), the Two-Rooms scene of the resolution study and
`data/sensitivity/env_two_rooms.npz` read `two_rooms.png` as it stands, rows as `x`, without the transpose and flip that
`from_image` and Bench-MR apply. The two grids differ by a quarter turn, so the
same coordinates name different places in each: the endpoints `(30, 70)` and
`(90, 80)` of the sweeps are free in both grids, but they do not pose the same
problem.
