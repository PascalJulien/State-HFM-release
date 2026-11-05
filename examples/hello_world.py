"""Plan one path on a map of the repository and draw it.

    python examples/hello_world.py

Needs a CUDA GPU; Bench-MR is not needed. The figure is written to
figures/hello_world.png, or to $SHFM_FIGURE_DIR when it is set.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)            # works from a clone, installed or not

import matplotlib.pyplot as plt

from shfm import from_image, Planner, REFERENCE_COEFFICIENTS
from shfm.figures import save
from shfm.plotting import draw_map, draw_path, draw_footprints, draw_pose

# The map and its scale: at zoom=4 the vehicle spans 14.6 grid units, one unit is 25 cm.
env = from_image(os.path.join(ROOT, "data", "maps", "two_rooms.png"), (100, 100), zoom=4)

# Poses are x, y in grid units, the heading in radians, and the mode (0: forward).
start, goal = [30, 70, 0.0, 0], [90, 80, 3.14, 0]
planner = Planner(env, k1=2, k2=3, **REFERENCE_COEFFICIENTS)
result = planner.solve(seed=start, tips=[goal], free_start_mode=True)
path = result.path(0)               # (n, 4): x, y, heading, mode, from start to goal

fig, ax = plt.subplots(figsize=(6.5, 6.5))
draw_map(ax, env)
draw_path(ax, path, by_state=True)
draw_footprints(ax, path, env.vehicle, env.zoom, n=14, color="0.45")
draw_pose(ax, *start[:3], env.vehicle, env.zoom, color="0.45")        # start in grey
draw_pose(ax, *goal[:3], env.vehicle, env.zoom, color="black")        # goal in black
ax.legend(loc="lower left", fontsize=9)  # the path is coloured by mode
print("%d points, figure written to %s" % (len(path), save(fig, "hello_world")[0]))
