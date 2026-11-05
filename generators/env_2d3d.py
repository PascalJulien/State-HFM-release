#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Workspace and configuration space.

(a) The 2D workspace W with its obstacles W_obs (black) and an oriented
    vehicle footprint.
(b) The 3D configuration space Omega = (x, y, theta): the slice at height
    theta is the configuration obstacle at that orientation, that is the 2D
    obstacle dilated by the rectangular footprint rotated by theta (a
    Minkowski sum). As theta varies, the dilated shape stretches and turns.

Shapely is not used: for each theta, the set of vehicle centres for which the
rectangle rotated by theta overlaps an obstacle is rasterised on an (x, y)
grid, with a polygon-polygon overlap test by separating axes (SAT). The
filled outline of each slice is drawn, semi-transparent and coloured by theta,
at height z = theta.

Writes:
  - env_2d3d_2d.png        : the 2D workspace
  - env_2d3d_3d.png        : the 3D configuration space
  - env_2d3d_combined.png  : (a) and (b) side by side
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

# Output goes to the repository's figures/ (override with SHFM_FIGURE_DIR).
import os as _os
_REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
_OUTDIR = _os.environ.get("SHFM_FIGURE_DIR", _os.path.join(_REPO, "figures"))
_os.makedirs(_OUTDIR, exist_ok=True)
from matplotlib.colors import Normalize
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

OUT = _OUTDIR

# ----------------------------------------------------------------------
# 1. 2D workspace
# ----------------------------------------------------------------------
Lx, Ly = 10.0, 8.0
Nx, Ny = 200, 160                      # rasterisation grid
xs = np.linspace(0, Lx, Nx)
ys = np.linspace(0, Ly, Ny)
XX, YY = np.meshgrid(xs, ys, indexing="ij")   # (Nx, Ny)

# 2D obstacles, kept simple for legibility. Each is a rectangle given by its
# centre, its half-sides and its own orientation (deg): an elongated bar and a
# small square.
#   (cx, cy, half_len, half_wid, ang_deg)
obstacles = [
    (3.2, 5.0, 1.3, 0.45, 20.0),   # O1: oblique bar
    (6.8, 3.0, 0.9, 0.9,   0.0),   # O2: square block
]
obst_base_colors = ["#1f6feb", "#d1432a"]

# ----------------------------------------------------------------------
# Vehicle: rectangular footprint, length 2.5, width 1.
# Reference point at the centre; half-length a, half-width b.
# ----------------------------------------------------------------------
a, b = 1.25, 0.50    # length 2.5, width 1.0


def rect_corners(cx, cy, hl, hw, ang):
    """Four corners (counter-clockwise) of a rectangle centred at (cx, cy),
    with half-sides (hl, hw) along its own axes, oriented by ang (rad)."""
    c, s = np.cos(ang), np.sin(ang)
    # own axes
    ux, uy = c, s          # length axis
    vx, vy = -s, c         # width axis
    pts = []
    for sl, sw in [(+1, +1), (-1, +1), (-1, -1), (+1, -1)]:
        px = cx + sl * hl * ux + sw * hw * vx
        py = cy + sl * hl * uy + sw * hw * vy
        pts.append((px, py))
    return np.array(pts)


def poly_axes(poly):
    """Unit normals to the edges of a convex polygon."""
    n = len(poly)
    axes = []
    for i in range(n):
        e = poly[(i + 1) % n] - poly[i]
        ax = np.array([-e[1], e[0]])
        nrm = np.hypot(ax[0], ax[1])
        if nrm > 1e-12:
            axes.append(ax / nrm)
    return axes


def sat_overlap(polyA, axesA, polyB, axesB):
    """Overlap of two convex polygons by separating axes.
    polyA is fixed; polyB is a batch (..., 4, 2) of polygons, one per centre.
    Returns a boolean mask over the batch (True = overlap)."""
    # polyA: (4,2); polyB: (M,4,2)
    M = polyB.shape[0]
    overlap = np.ones(M, dtype=bool)
    for ax in axesA + axesB:
        ax = np.asarray(ax)
        # projection of A (scalar min/max)
        pa = polyA @ ax
        amin, amax = pa.min(), pa.max()
        # projection of B (M,4) -> min/max over the four corners
        pb = polyB @ ax                      # (M,4)
        bmin = pb.min(axis=1)
        bmax = pb.max(axis=1)
        # separated along this axis -> no collision
        sep = (amax < bmin) | (bmax < amin)
        overlap &= ~sep
        if not overlap.any():
            break
    return overlap


# Corners and axes of the obstacles (fixed)
obst_polys = [rect_corners(cx, cy, hl, hw, np.radians(ad))
              for (cx, cy, hl, hw, ad) in obstacles]
obst_axes = [poly_axes(p) for p in obst_polys]

# Grid of centres (flattened), reused for every theta
cx_flat = XX.ravel()
cy_flat = YY.ravel()
Ncell = cx_flat.size

# offsets of the four corners of the vehicle, per theta
def robot_polys_batch(theta):
    """For one theta, return (Ncell,4,2): the vehicle rectangle rotated by
    theta, placed at every centre of the grid."""
    c, s = np.cos(theta), np.sin(theta)
    ux, uy = c, s
    vx, vy = -s, c
    # corner offsets from the centre
    offs = []
    for sl, sw in [(+1, +1), (-1, +1), (-1, -1), (+1, -1)]:
        ox = sl * a * ux + sw * b * vx
        oy = sl * a * uy + sw * b * vy
        offs.append((ox, oy))
    offs = np.array(offs)                       # (4,2)
    poly = np.empty((Ncell, 4, 2))
    poly[:, :, 0] = cx_flat[:, None] + offs[None, :, 0]
    poly[:, :, 1] = cy_flat[:, None] + offs[None, :, 1]
    return poly


# ----------------------------------------------------------------------
# 2. Configuration space: configuration obstacle (Minkowski) at each theta
# ----------------------------------------------------------------------
Ntheta = 19
thetas = np.linspace(0.0, np.radians(360.0), Ntheta, endpoint=True)  # 0..360 deg, 20 deg step

# Cobs[:, :, k] = True when the vehicle at orientation thetas[k], centred
# there, overlaps an obstacle
Cobs = np.zeros((Nx, Ny, Ntheta), dtype=bool)
for k, th in enumerate(thetas):
    rpoly = robot_polys_batch(th)               # (Ncell,4,2)
    # axes of the vehicle at this theta: two distinct directions (rectangle)
    c, s = np.cos(th), np.sin(th)
    rob_axes = [np.array([c, s]), np.array([-s, c])]
    hit = np.zeros(Ncell, dtype=bool)
    for op, oax in zip(obst_polys, obst_axes):
        hit |= sat_overlap(op, oax, rpoly, rob_axes)
    Cobs[:, :, k] = hit.reshape(Nx, Ny)

# ----------------------------------------------------------------------
# 3. Panel (a): 2D workspace
# ----------------------------------------------------------------------
def draw_obstacles(ax):
    for (cx, cy, hl, hw, ad), c in zip(obstacles, obst_base_colors):
        r = Rectangle((cx - hl, cy - hw), 2 * hl, 2 * hw,
                      facecolor="black", edgecolor="black", lw=2.5, zorder=3)
        r.set_transform(Affine2D().rotate_deg_around(cx, cy, ad) + ax.transData)
        ax.add_patch(r)


def make_2d(ax):
    ax.set_facecolor("white")
    draw_obstacles(ax)

    # oriented vehicle footprint (what dilates the obstacle)
    rx, ry, rth = 4.6, 1.8, np.radians(35)
    rob = Rectangle((rx - a, ry - b), 2 * a, 2 * b,
                    facecolor="0.75", edgecolor="0.2", lw=1.2, alpha=0.9, zorder=4)
    rob.set_transform(Affine2D().rotate_around(rx, ry, rth) + ax.transData)
    ax.add_patch(rob)
    ax.annotate(r"footprint robot $\theta$", (rx, ry), (rx + 1.0, ry - 1.1),
                fontsize=8.5, color="0.2",
                arrowprops=dict(arrowstyle="->", color="0.3", lw=0.8))

    for n, (cx, cy, hl, hw, ad) in enumerate(obstacles, start=1):
        ax.text(cx, cy + hl + 0.45, f"O{n}", ha="center",
                fontsize=10, fontweight="bold", color="0.1")

    ax.set_xlim(-0.3, Lx + 0.3)
    ax.set_ylim(-0.3, Ly + 0.3)
    ax.set_aspect("equal")
    # keep the frame (spines visible)
    ax.set_frame_on(True)
    ax.set_xlabel(r"$x$")
    ax.set_ylabel(r"$y$")
    ax.set_title(r"(a) Espace de travail $\mathcal{W}$ et obstacles $\mathcal{W}_{obs}$"
                 r" (en noir)", fontsize=12)


# ----------------------------------------------------------------------
# 4. Panel (b): 3D configuration space, coloured by theta
# ----------------------------------------------------------------------
cmap = cm.get_cmap("twilight")   # cyclic colour map, suited to theta in 0-360
norm = Normalize(vmin=0.0, vmax=360.0)


def make_3d(ax, fig):
    """Stack the slices: the slice at z = theta is the configuration obstacle
    at that orientation (Minkowski sum of the 2D obstacle and the vehicle
    rotated by theta), so the dilated shape turns and stretches with theta."""
    thg = np.degrees(thetas)
    for k in range(Ntheta):
        sl = Cobs[:, :, k].astype(float)
        if sl.max() == 0:
            continue
        z = thg[k]
        col = cmap(norm(z))
        ax.contourf(XX, YY, sl, levels=[0.5, 1.5], zdir="z", offset=z,
                    colors=[col], alpha=0.42, antialiased=True)
        ax.contour(XX, YY, sl, levels=[0.5], zdir="z", offset=z,
                   colors=[col], linewidths=0.9, alpha=0.95)

    # the bare 2D obstacles on the floor (z = 0), dark grey, to link with
    # panel (a)
    walls2d = np.zeros((Nx, Ny), dtype=bool)
    for op, oax in zip(obst_polys, obst_axes):
        # point-in-polygon test: a centre is inside when it lies on the inner
        # side of every edge
        inside = np.ones(Ncell, dtype=bool)
        n = len(op)
        for i in range(n):
            e = op[(i + 1) % n] - op[i]
            rel = np.stack([cx_flat - op[i, 0], cy_flat - op[i, 1]], axis=1)
            cross = e[0] * rel[:, 1] - e[1] * rel[:, 0]
            inside &= (cross >= -1e-9)
        walls2d |= inside.reshape(Nx, Ny)
    ax.contourf(XX, YY, walls2d.astype(float), levels=[0.5, 1.5],
                zdir="z", offset=0, colors=["0.12"], alpha=0.9)

    for n, (cx, cy, hl, hw, ad) in enumerate(obstacles, start=1):
        ax.text(cx, cy, 375, f"O{n}", ha="center",
                fontsize=10, fontweight="bold", color="0.1")

    ax.set_xlim(0, Lx)
    ax.set_ylim(0, Ly)
    ax.set_zlim(0, 360)
    ax.set_xlabel(r"$x$", labelpad=10)
    ax.set_ylabel(r"$y$", labelpad=10)
    ax.set_zlabel(r"$\theta$ (deg)", labelpad=10)
    ax.set_title(r"(b) Espace de configuration $\Omega$ et $\Omega_{obs}$",
                 fontsize=12)
    ax.view_init(elev=14, azim=-58)
    try:
        ax.set_box_aspect((Lx, Ly, 16.0))
    except Exception:
        pass
    ax.set_zticks([0, 90, 180, 270, 360])

    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax, shrink=0.55, pad=0.10,
                      ticks=[0, 90, 180, 270, 360])
    cb.set_label(r"orientation $\theta$ (deg)")


# ----------------------------------------------------------------------
# 5. Rendering
# ----------------------------------------------------------------------
def render_2d():
    fig, ax = plt.subplots(figsize=(6.0, 5.0))
    make_2d(ax)
    fig.tight_layout()
    p = os.path.join(OUT, "env_2d3d_2d.png")
    fig.savefig(p, dpi=170, bbox_inches="tight")
    plt.close(fig)
    return p


def render_3d():
    fig = plt.figure(figsize=(7.5, 6.0))
    ax = fig.add_subplot(111, projection="3d")
    make_3d(ax, fig)
    p = os.path.join(OUT, "env_2d3d_3d.png")
    fig.savefig(p, dpi=170, bbox_inches="tight")
    plt.close(fig)
    return p


def render_combined():
    fig = plt.figure(figsize=(13.5, 5.6))
    ax1 = fig.add_subplot(1, 2, 1)
    make_2d(ax1)
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    make_3d(ax2, fig)
    p = os.path.join(OUT, "env_2d3d_combined.png")
    fig.savefig(p, dpi=170, bbox_inches="tight")
    plt.close(fig)
    return p


if __name__ == "__main__":
    p1 = render_2d()
    p2 = render_3d()
    p3 = render_combined()
    print("wrote:", p1)
    print("wrote:", p2)
    print("wrote:", p3)
