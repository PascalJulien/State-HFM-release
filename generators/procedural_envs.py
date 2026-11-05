#!/usr/bin/env python3
"""Four of the procedurally generated corridor environments.

Bench-MR generates the corridor maps from a seed
(``set_corridor_grid_env(200, 200, branches=100, radius=8)``), so they are part
of the campaign result file rather than an input to it. We read the first
four runs back from the campaign and draw their occupancy grids.
"""
import gzip
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.environ.get("SHFM_FIGURE_DIR", os.path.join(_REPO, "figures"))
os.makedirs(OUT, exist_ok=True)

CAMPAIGN = os.path.join(_REPO, "data", "campaigns", "procedural.json.gz")
N_SHOWN = 4


def occupancy(env):
    """Bench-MR stores a grid as one string of 0/1, row by row from the bottom."""
    w = env["width"]
    flat = env["map"]
    return np.array([int(c) for c in flat]).reshape(len(flat) // w, w)


with gzip.open(CAMPAIGN, "rt") as fh:
    runs = json.load(fh)["runs"]

# Panels on a 1x5 grid, the empty fifth dropped by the tight bounding box; this
# sets the aspect ratio of the panels.
fig = plt.figure(figsize=(4 * 5, 5))
for i in range(N_SHOWN):
    ax = plt.subplot(1, 5, i + 1)
    grid = occupancy(runs[i]["environment"])
    # imshow with no extent, the axes then stretched to the size of the map.
    # Each map is shown flipped top for bottom against the grid the planners
    # were given: Bench-MR numbers the rows of a grid from the bottom, and
    # plot_env, which draws the other maps, flips accordingly. Nothing reads
    # this figure back.
    ax.imshow(1 - np.flipud(grid), cmap="gray", origin="lower",
              interpolation="nearest")
    ax.set_xlim(0, grid.shape[1])
    ax.set_ylim(0, grid.shape[0])

fig.savefig(os.path.join(OUT, "procedural_envs.pdf"), bbox_inches="tight")
fig.savefig(os.path.join(OUT, "procedural_envs.png"), bbox_inches="tight", dpi=150)
print("procedural_envs.pdf")
