#!/usr/bin/env python3
"""Tissot's indicatrix of the forklift, drawn as a shell in (x', y', theta').

Each edge is the path of the rear axle centre over unit length under one
extreme steering combination. The 4WS shell (blue) joins the four
combinations of the front and rear limits, the 2WS surface (black) the two
limits of a single axle, and the red sphere is the indicatrix of a holonomic
vehicle, for comparison.

Writes tissot_d_improved.png and tissot_d_improved.pdf.
"""
import numpy as np
import matplotlib as mpl
mpl.use('Agg')
mpl.rcParams['font.family'] = 'DejaVu Sans'
mpl.rcParams['savefig.dpi'] = 200
mpl.rcParams['savefig.bbox'] = 'tight'
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from pathlib import Path

# Output goes to the repository's figures/ (override with SHFM_FIGURE_DIR).
import os as _os
_REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
_OUTDIR = _os.environ.get("SHFM_FIGURE_DIR", _os.path.join(_REPO, "figures"))
_os.makedirs(_OUTDIR, exist_ok=True)

# ---- parameters ------------------------------------------------------------
psi_max = 0.5
phi_max = 0.5
wheelbase = 1.38
tau = 0.1
length = 1.0
steering_phi = [phi_max, -phi_max]   # front
steering_psi = [psi_max, -psi_max]   # rear


def kinematic(q, psi, phi, tau, wb):
    # cos(psi): rolling without slip, tau being the distance travelled by the
    # rear axle centre.
    return [q[0] + tau * np.cos(q[2] + psi),
            q[1] + tau * np.sin(q[2] + psi),
            q[2] + tau * np.cos(psi)
            * (np.tan(phi) - np.tan(psi)) / wb]


def integrate(psi, phi):
    q = [0.0, 0.0, 0.0]
    path = [q]
    for _ in range(int(length / tau)):
        q = kinematic(q, psi, phi, tau, wheelbase)
        path.append(q)
    return np.array(path)


# 4WS: the four extreme combinations give the four edges of the pyramid, in
# cyclic order (one sign change between neighbours) so that the faces join up
combos = [(phi_max, psi_max), (phi_max, -psi_max),
          (-phi_max, -psi_max), (-phi_max, psi_max)]
paths4 = [integrate(psi, phi) for (phi, psi) in combos]

# 2WS: phi = 0, psi at its limits
paths2 = [integrate(psi, 0.0) for psi in steering_psi]


def ribbon(a, b):
    """Ribbon surface between two curves a and b."""
    verts = np.vstack([a, b[::-1]])
    return verts


fig = plt.figure(figsize=(8.5, 8.5))
ax = fig.add_subplot(111, projection='3d')

# --- 4WS shell (blue): four side faces between adjacent edges ---------------
for i in range(len(paths4)):
    face = ribbon(paths4[i], paths4[(i + 1) % len(paths4)])
    poly = Poly3DCollection([face], alpha=0.18)
    poly.set_facecolor('royalblue'); poly.set_edgecolor('none')
    ax.add_collection3d(poly)
for p in paths4:
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color='royalblue', lw=2.0)

# --- 2WS surface (black) -----------------------------------------------------
face2 = ribbon(paths2[0], paths2[1])
poly2 = Poly3DCollection([face2], alpha=0.22)
poly2.set_facecolor('black'); poly2.set_edgecolor('none')
ax.add_collection3d(poly2)
for p in paths2:
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color='black', lw=2.0)

# --- unit sphere of a holonomic vehicle (translucent red) --------------------
u = np.linspace(0, 2 * np.pi, 60)
v = np.linspace(0, np.pi, 60)
xs = np.outer(np.cos(u), np.sin(v))
ys = np.outer(np.sin(u), np.sin(v))
zs = np.outer(np.ones_like(u), np.cos(v))
ax.plot_surface(xs, ys, zs, color='red', alpha=0.10, linewidth=0)

# --- axes, labels, view ------------------------------------------------------
ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2); ax.set_zlim(-1.2, 1.2)
ax.set_box_aspect((1, 1, 1), zoom=0.86)
ax.set_xlabel(r'$\dot{x}$', fontsize=18, labelpad=10)
ax.set_ylabel(r'$\dot{y}$', fontsize=18, labelpad=10)
ax.set_zlabel(r'$\dot{\theta}$', fontsize=18, labelpad=14)
ax.tick_params(labelsize=11)
ax.view_init(elev=22, azim=-58)

legend = [
    Patch(facecolor='royalblue', alpha=0.45, label='Modèle 4WS'),
    Patch(facecolor='black', alpha=0.5, label='Modèle 2WS'),
    Patch(facecolor='red', alpha=0.25, label='Sphère unité (véhicule holonome)'),
]
ax.legend(handles=legend, loc='upper left', fontsize=12, framealpha=0.95)

fig.subplots_adjust(left=0.02, right=0.86, top=0.98, bottom=0.04)
out = Path(_OUTDIR)
fig.savefig(out / 'tissot_d_improved.png')
fig.savefig(out / 'tissot_d_improved.pdf')
print('OK ->', out / 'tissot_d_improved.png')
