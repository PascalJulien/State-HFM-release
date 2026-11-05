#!/usr/bin/env python3
"""Collision test at one position (x_r, y_r), for two orientations theta.

The configuration obstacle, the real obstacle dilated by the vehicle footprint
oriented at theta, changes shape with theta. The fixed point (x_r, y_r) is
outside it at theta = 0 (no collision) and inside it at theta = 10 deg
(collision).

Writes collision_check.png and collision_check.pdf.
"""
import numpy as np
import matplotlib as mpl
mpl.use('Agg')
mpl.rcParams['font.family'] = 'DejaVu Sans'
mpl.rcParams['savefig.dpi'] = 200
mpl.rcParams['savefig.bbox'] = 'tight'
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
from pathlib import Path

# Output goes to the repository's figures/ (override with SHFM_FIGURE_DIR).
import os as _os
_REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
_OUTDIR = _os.environ.get("SHFM_FIGURE_DIR", _os.path.join(_REPO, "figures"))
_os.makedirs(_OUTDIR, exist_ok=True)

OUT = __import__('pathlib').Path(_OUTDIR)

# --- geometry ----------------------------------------------------------------
ROB_L, ROB_W = 2.6, 1.0          # vehicle footprint (length x width)
XR, YR = 0.0, 0.85               # reference position (fixed)
# obstacle: a horizontal wall above
OBS = np.array([[-3.0, 1.45], [3.0, 1.45], [3.0, 3.2], [-3.0, 3.2]])


def rect(cx, cy, w, h, theta):
    c = np.array([[-w / 2, -h / 2], [w / 2, -h / 2],
                  [w / 2, h / 2], [-w / 2, h / 2]])
    ct, st = np.cos(theta), np.sin(theta)
    R = np.array([[ct, -st], [st, ct]])
    return c @ R.T + np.array([cx, cy])


def sat_overlap(A, B):
    """True when the convex polygons A and B overlap (separating axes)."""
    for poly in (A, B):
        n = len(poly)
        for i in range(n):
            e = poly[(i + 1) % n] - poly[i]
            ax = np.array([-e[1], e[0]])
            ax = ax / (np.linalg.norm(ax) + 1e-12)
            pa = A @ ax
            pb = B @ ax
            if pa.max() < pb.min() - 1e-9 or pb.max() < pa.min() - 1e-9:
                return False
    return True


def cobstacle_mask(theta, gx, gy):
    """Boolean mask of the (x, y) where the vehicle, oriented at theta and
    centred there, hits the obstacle: the configuration obstacle at theta."""
    M = np.zeros((len(gy), len(gx)), bool)
    for j, y in enumerate(gy):
        for i, x in enumerate(gx):
            if sat_overlap(rect(x, y, ROB_L, ROB_W, theta), OBS):
                M[j, i] = True
    return M


def panel(ax, theta_deg, title_letter):
    th = np.deg2rad(theta_deg)
    gx = np.linspace(-4.5, 4.5, 240)
    gy = np.linspace(-2.2, 3.6, 170)
    M = cobstacle_mask(th, gx, gy)
    # configuration obstacle (the dilated obstacle, which changes shape)
    ax.contourf(gx, gy, M, levels=[0.5, 1.5], colors=['#f4b6b6'], alpha=0.65)
    ax.contour(gx, gy, M, levels=[0.5], colors=['#c0392b'],
               linewidths=1.4, linestyles='--')
    # real obstacle
    ax.add_patch(MplPoly(OBS, closed=True, facecolor='0.25', edgecolor='k', zorder=3))
    # vehicle at (x_r, y_r), oriented at theta
    rob = rect(XR, YR, ROB_L, ROB_W, th)
    collide = sat_overlap(rob, OBS)
    ax.add_patch(MplPoly(rob, closed=True, facecolor='0.82',
                         edgecolor='k', lw=1.4, zorder=4))
    # reference point
    ax.plot([XR], [YR], 'o', color='red', ms=7, zorder=6)
    ax.annotate(r'$(x_r,\,y_r)$', (XR, YR), textcoords='offset points',
                xytext=(8, -14), fontsize=12, color='red')
    # collision label
    if collide:
        ax.text(0.5, -0.13, 'Collision', transform=ax.transAxes, ha='center',
                fontsize=14, color='#c0392b', weight='bold',
                bbox=dict(boxstyle='round', fc='#fdecea', ec='#c0392b'))
    else:
        ax.text(0.5, -0.13, 'Pas de collision', transform=ax.transAxes,
                ha='center', fontsize=14, color='#1e8449', weight='bold',
                bbox=dict(boxstyle='round', fc='#eafaf1', ec='#1e8449'))
    ax.set_title(r'(%s) $\theta = %d^\circ$' % (title_letter, theta_deg),
                 fontsize=14)
    ax.set_xlim(-4.5, 4.5); ax.set_ylim(-2.2, 3.6)
    ax.set_aspect('equal'); ax.set_xlabel('x'); ax.set_ylabel('y')
    ax.grid(alpha=0.25)
    return collide


fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
c0 = panel(axes[0], 0, 'a')
c1 = panel(axes[1], 10, 'b')
# shared legend
from matplotlib.patches import Patch
axes[1].legend(handles=[
    Patch(facecolor='0.25', label='obstacle $\\mathcal{W}_{obs}$'),
    Patch(facecolor='#f4b6b6', edgecolor='#c0392b', label="obstacle de configuration ($\\theta$)"),
    Patch(facecolor='0.82', edgecolor='k', label='empreinte robot'),
], loc='upper right', fontsize=10)
fig.suptitle("L'obstacle de configuration (dilaté par l'empreinte orientée) "
             "change de forme avec $\\theta$", fontsize=13)
fig.tight_layout(rect=[0, 0.02, 1, 0.96])
fig.savefig(OUT / 'collision_check.png')
fig.savefig(OUT / 'collision_check.pdf')
print('OK collision : theta=0 collide=%s | theta=10 collide=%s' % (c0, c1))
