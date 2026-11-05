#!/usr/bin/env python3
"""The adaptive stencil of the worked example, in 3D.

The five active offsets and their weights are those of the Selling
decomposition of the relaxed metric of the control vector ``u_1``, which
``notebooks/09_scheme_step_by_step.ipynb`` computes; the sixth, ``g_23``, has
weight zero. Every offset is oriented towards ``u_1``. The view angles are
fixed in :data:`VIEW`.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.environ.get("SHFM_FIGURE_DIR", os.path.join(_REPO, "figures"))
os.makedirs(OUT, exist_ok=True)

# View angles of the figure.
VIEW = dict(elev=20, azim=-60)

# Preferred direction u_1 of the example.
v1 = np.array([np.sqrt(2) / 2, np.sqrt(2) / 2, 0.5])

# Offsets g_ij and Selling weights rho_ij, signed as the decomposition gives them.
stencil_raw = [
    ("$g_{01}$", np.array([3, 3, 2]), 0.030),
    ("$g_{02}$", np.array([2, 2, 1]), 0.022),
    ("$g_{03}$", np.array([1, 2, 1]), 0.010),
    ("$g_{12}$", np.array([-1, -1, -1]), 0.095),
    ("$g_{13}$", np.array([-2, -1, -1]), 0.010),
    ("$g_{23}$", np.array([-1, 0, 0]), 0.000),
]
# Every offset points towards v1.
stencil = [(lab, g if np.dot(g, v1) >= 0 else -g, rho)
           for lab, g, rho in stencil_raw]

lim = 3.5
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")

# Integer points of the grid, faint.
c = np.arange(int(np.floor(-lim)) + 1, int(np.ceil(lim)))
xx, yy, zz = np.meshgrid(c, c, c)
ax.scatter(xx, yy, zz, s=6, color="lightgray", alpha=0.34)

# The point q itself, its label offset to the left so the arrows stay readable.
ax.scatter([0], [0], [0], color="black", s=25)
ax.text(-0.45, -0.30, 0.05, "q", color="black", fontsize=15, ha="right", va="top")


def add_vector(p, color, lw, label):
    p = np.asarray(p, float)
    ax.quiver(0, 0, 0, p[0], p[1], p[2],
              color=color, linewidth=lw, arrow_length_ratio=0.18)
    ext = 0.30 / np.linalg.norm(p)
    ax.text(p[0] * (1 + ext), p[1] * (1 + ext), p[2] * (1 + ext),
            label, color=color, fontsize=13)


add_vector(2.5 * v1, "red", 2.5, "$u_1$")

for lab, g, rho in stencil:
    add_vector(g, "royalblue" if rho > 0 else "gray", 1.5, lab)

ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-lim, lim)
ax.set_xlabel("x", fontsize=17)
ax.set_ylabel("y", fontsize=17)
ax.set_zlabel("θ", fontsize=17)
ax.set_box_aspect((1, 1, 1))
ax.view_init(**VIEW)

fig.savefig(os.path.join(OUT, "stencil.png"))
fig.savefig(os.path.join(OUT, "stencil.pdf"))
print("stencil.png")
