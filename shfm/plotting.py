"""Drawing conventions used throughout the notebooks.

Three rules are applied everywhere.

* Axes are in grid units, and the label states what one unit is worth in
  metres.
* The vehicle footprint is drawn as a dashed rectangle.
* Start is blue, goal is red, and a small nose marks the heading; over a
  path coloured by mode, start and goal are grey and black instead.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from .states import SOLVER_ORDER

__all__ = ["axis_labels", "draw_map", "draw_path", "draw_footprints",
           "draw_pose", "overlay_by_value", "STATE_COLORS"]

STATE_COLORS = {
    "nav_fwd": "tab:blue",
    "nav_bwd": "tab:orange",
    "man_fwd": "tab:red",
    "man_bwd": "tab:purple",
}
"""One colour per operational mode, for geodesics coloured by state."""


def _xytheta(path):
    p = np.asarray(path, dtype=float)
    if p.shape[0] in (3, 4) and p.shape[0] <= p.shape[1]:
        p = p.T
    return p


def axis_labels(ax, zoom, fontsize=13):
    """Label both axes in grid units, with the conversion to metres."""
    note = "1 unit = %.3g m" % (1.0 / float(zoom))
    ax.set_xlabel("$x$ (units ; %s)" % note, fontsize=fontsize)
    ax.set_ylabel("$y$ (units ; %s)" % note, fontsize=fontsize)


def draw_map(ax, env, alpha=0.55):
    """Grey obstacles, equal aspect, axes in units."""
    ax.set_aspect("equal")
    ax.pcolormesh(env.X[0], env.X[1], env.walls.astype(float),
                  cmap="Greys", vmin=0.0, vmax=1.7, alpha=alpha,
                  shading="auto")
    axis_labels(ax, env.zoom)
    ax.set_xlim(env.X[0].min(), env.X[0].max())
    ax.set_ylim(env.X[1].min(), env.X[1].max())
    ax.grid(True, alpha=0.3)
    return ax


def draw_path(ax, path, color="tab:blue", lw=2.2, by_state=False, **kwargs):
    """Draw a geodesic, optionally coloured by the mode in use."""
    p = _xytheta(path)
    if not by_state or p.shape[1] < 4:
        return ax.plot(p[:, 0], p[:, 1], color=color, lw=lw,
                       solid_capstyle="round", zorder=3, **kwargs)
    s = p[:, 3].round().astype(int)
    for k, name in enumerate(SOLVER_ORDER):
        if (s == k).any():
            x, y = np.where(s == k, p[:, 0], np.nan), np.where(s == k, p[:, 1], np.nan)
        else:
            # An all-NaN line of more than 1000 points breaks the legend
            # placement of matplotlib 3.7; an empty one keeps the entry.
            x, y = [], []
        ax.plot(x, y, color=STATE_COLORS[name], lw=lw, solid_capstyle="round",
                zorder=3, label=name)
    return ax


def draw_footprints(ax, path, vehicle, zoom, n=18, color="tab:blue", alpha=0.9):
    """Dashed footprints, evenly spaced along the path."""
    p = _xytheta(path)
    idx = np.unique(np.linspace(0, len(p) - 1, max(2, n)).astype(int))
    for i in idx:
        poly = vehicle.footprint_polygon(p[i, 0], p[i, 1], p[i, 2], zoom)
        ax.add_patch(Polygon(poly, closed=True, fill=False, linestyle="--",
                             edgecolor=color, lw=1.2, alpha=alpha, zorder=2))
    return ax


def draw_pose(ax, x, y, theta, vehicle, zoom, color="tab:blue", nose=True):
    """A single filled footprint, with an arrowhead inside its front end.

    The arrowhead gives the heading and stays within the footprint, so a pose
    drawn against a wall does not appear to touch it.
    """
    poly = vehicle.footprint_polygon(x, y, theta, zoom)
    ax.add_patch(Polygon(poly, closed=True, fill=True, facecolor=color,
                         alpha=0.30, edgecolor=color, lw=2.2, zorder=6))
    if nose:
        # Local frame of footprint_polygon before its rotation: the front
        # lies on negative x.
        front = (vehicle.length - vehicle.rear_axle_offset) * zoom
        half_w = 0.5 * vehicle.width * zoom
        tip = -front + 0.15 * half_w
        base = tip + 1.6 * half_w
        local = np.array([[base, -0.7 * half_w], [base, 0.7 * half_w],
                          [tip, 0.0]])
        th = theta + np.pi
        c, s = np.cos(th), np.sin(th)
        rot = np.array([[c, s], [-s, c]])
        ax.add_patch(Polygon(local @ rot + [x, y], closed=True, fill=True,
                             facecolor=color, edgecolor=color, lw=1.0,
                             zorder=7))
    return ax


def overlay_by_value(ax, env, paths, values, label, cmap="viridis",
                     start=None, goal=None):
    """Superimpose paths, one colour per parameter value.

    Colouring by a performance score instead would give the same colour to two
    settings whose behaviour is opposite, whenever their scores coincide.

    Line width decreases with the index so that settings producing the same
    path remain visible as a stack rather than hiding one another.
    """
    import matplotlib.colors as mcolors
    import matplotlib.cm as cm

    draw_map(ax, env)
    n = len(paths)
    base = plt.get_cmap(cmap)
    colors = [base(i / max(1, n - 1)) for i in range(n)]
    for i, path in enumerate(paths):
        p = _xytheta(path)
        ax.plot(p[:, 0], p[:, 1], color=colors[i],
                lw=4.0 - 2.2 * i / max(1, n - 1),
                solid_capstyle="round", zorder=3 + i)
    if start is not None:
        draw_pose(ax, *start, env.vehicle, env.zoom, "tab:blue")
    if goal is not None:
        draw_pose(ax, *goal, env.vehicle, env.zoom, "tab:red")

    norm = mcolors.BoundaryNorm(np.arange(n + 1) - 0.5, n)
    sm = cm.ScalarMappable(norm=norm, cmap=mcolors.ListedColormap(colors))
    cb = ax.figure.colorbar(sm, ax=ax, fraction=0.046, pad=0.03,
                            ticks=np.arange(n))
    fmt = lambda v: "%d" % v if abs(v - round(v)) < 1e-9 else ("%.2f" % v).rstrip("0").rstrip(".")
    cb.ax.set_yticklabels([fmt(v) for v in values], fontsize=10)
    cb.set_label(label, fontsize=14)
    return ax
