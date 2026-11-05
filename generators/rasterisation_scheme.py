"""Schematic of how a coarse grid closes Narrow-Passage.

(a) cells of 12.5 cm aligned with the walls: the passage keeps its 1.00 m and
    the envelope of the covering circles (0.90 m) goes through;
(b) cells of 20 cm, shifted: every cell that touches the real obstacle is
    marked, the passage falls to 0.80 m and the circles touch marked cells;
(c) the extraction along the edge of a corridor of the configuration space:
    the direction is interpolated in the current cell from its free corners
    only, as agd does. The directions point towards the opening only at the
    last two nodes of the corridor, where the diagonal to it is clear; the path
    turns there, runs into the wall in the last cell of the corridor and stops
    in a cell with no free corner.

Walls straight, vehicle centred. Writes ``np_rasterisation``. CPU only.
"""
import os

import numpy as np
from matplotlib.patches import Rectangle, Circle, Patch
from matplotlib.lines import Line2D

import study_tools as S
from study_tools import plt
from shfm import DEFAULT_VEHICLE as V

W = 1.00                                  # real passage (m)
R = V.ball_radius(1.0)                    # radius of the circles (m)
X0, X1, Y0, Y1 = 0.0, 2.4, -0.45, 1.45
GREY = "0.70"


def schema(ax, h, off, title):
    # marked cells: every cell that touches the real obstacle
    ys = np.arange(off - 10 * h, Y1 + h, h)
    xs = np.arange(X0, X1 + h, h)
    for y in ys:
        if y + h <= Y0 or y >= Y1:
            continue
        if y < 0.0 - 1e-9 or y + h > W + 1e-9:           # touches a wall
            ax.add_patch(Rectangle((X0, y), X1 - X0, h, color=GREY, lw=0, zorder=1))
    yv = [y for y in ys if Y0 - 1e-9 <= y <= Y1 + 1e-9]
    for x in xs:
        if x <= X1 + 1e-9:
            ax.plot([x, x], [Y0, Y1], color="0.55", lw=0.5, zorder=2)
    for y in yv:
        ax.plot([X0, X1], [y, y], color="0.55", lw=0.5, zorder=2)
    # real obstacle
    for y0, y1 in ((Y0, 0.0), (W, Y1)):
        ax.add_patch(Rectangle((X0, y0), X1 - X0, y1 - y0, fill=False, hatch="////",
                               edgecolor="k", lw=0, zorder=3))
    ax.plot([X0, X1], [0.0, 0.0], color="k", lw=1.6, zorder=4)
    ax.plot([X0, X1], [W, W], color="k", lw=1.6, zorder=4)
    # free passage once the cells are marked
    lo = min(y + h for y in ys if y < 0.0 - 1e-9 and y + h > 0.0 - 1e-9) if off % h else 0.0
    lo = max(lo, 0.0)
    hi = max(y for y in ys if y + h > W + 1e-9 and y < W + 1e-9) if off % h else W
    hi = min(hi, W)
    # vehicle: body and circles, centred in the passage
    yc = W / 2
    cx = np.linspace(0.45, X1 - 0.45, 6)
    hit = (yc - R < lo - 1e-9) or (yc + R > hi + 1e-9)
    col = "C3" if hit else "C0"
    for c in cx:
        ax.add_patch(Circle((c, yc), R, color=col, alpha=0.13, lw=0, zorder=5))
        ax.add_patch(Circle((c, yc), R, fill=False, edgecolor=col, lw=0.8, zorder=5))
    ax.add_patch(Rectangle((0.15, yc - V.width / 2), X1 - 0.3, V.width, fill=False,
                           edgecolor="C0", lw=2.0, zorder=6))
    # dimensions
    def cote(x, a, b, txt, color, side="right"):
        ax.annotate("", (x, a), (x, b), arrowprops=dict(arrowstyle="<->", color=color, lw=1.3),
                    zorder=8)
        ax.text(x + (0.04 if side == "right" else -0.04), (a + b) / 2, txt, color=color,
                fontsize=9, va="center", ha="left" if side == "right" else "right", zorder=8,
                bbox=dict(fc="white", ec="none", alpha=0.85, pad=0.8))
    fr = lambda v: ("%.2f" % v).replace(".", ",")
    cote(X0 - 0.08, yc - V.width / 2, yc + V.width / 2, "chariot\n%s m" % fr(V.width), "C0",
         side="left")
    cote(X1 + 0.08, yc - R, yc + R, "cercles\n%s m" % fr(2 * R), col)
    cote(X1 + 0.82, lo, hi, "passage\n%s m" % fr(hi - lo), "k")
    ax.set_xlim(X0 - 0.55, X1 + 1.32); ax.set_ylim(Y0, Y1)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(title, fontsize=11)


def schema_c(ax):
    """Stop in the obstacle. A corridor two nodes wide opens upwards on the
    left, towards the start. Each free node carries the direction of descent
    towards the start (arrows); the nodes in the obstacle carry none. The curve
    is integrated by interpolating, in the current cell, the directions of the
    free corners only, as agd's extraction does; it stops when the cell has no
    free corner left."""
    nx, ny = 10, 7
    free = np.zeros((nx, ny), bool)
    free[:, 1:3] = True                       # corridor: two free rows
    free[0:2, :] = True                       # opening upwards, on the left
    flow = np.zeros((nx, ny, 2))
    for i in range(nx):
        for j in range(ny):
            if not free[i, j]:
                continue
            if i <= 1:
                v = (-0.25, 1.0)              # in the opening: towards the start
            elif i <= 3:                      # last cells of the corridor: upwards once
                v = (-1.0, 0.75 * (4 - i) / 2 + 0.15)   # the diagonal to the opening is clear
            else:
                v = (-1.0, 0.0)
            flow[i, j] = np.array(v) / np.hypot(*v)
    for i in range(nx):
        ax.plot([i, i], [0, ny - 1], color="0.85", lw=0.6, zorder=1)
    for j in range(ny):
        ax.plot([0, nx - 1], [j, j], color="0.85", lw=0.6, zorder=1)
    X, Y = np.meshgrid(range(nx), range(ny), indexing="ij")
    ax.scatter(X[~free], Y[~free], s=34, c="k", zorder=4)
    ax.scatter(X[free], Y[free], s=38, facecolors="white", edgecolors="k", zorder=4)
    for i in range(nx):
        for j in range(ny):
            if free[i, j]:
                v = flow[i, j]
                ax.annotate("", (i + 0.62 * v[0], j + 0.62 * v[1]), (i + 0.14 * v[0], j + 0.14 * v[1]),
                            arrowprops=dict(arrowstyle="-|>", color="0.4", lw=1.1,
                                            mutation_scale=8, shrinkA=0, shrinkB=0), zorder=3)
    # integration: bilinear interpolation over the free corners only
    x = np.array([9.3, 2.3]); pts = [x.copy()]; h = 0.02
    for _ in range(2000):
        i, j = int(np.floor(x[0])), int(np.floor(x[1]))
        fx, fy = x[0] - i, x[1] - j
        acc, wsum = np.zeros(2), 0.0
        for di, dj, w in ((0, 0, (1 - fx) * (1 - fy)), (1, 0, fx * (1 - fy)),
                          (0, 1, (1 - fx) * fy), (1, 1, fx * fy)):
            ii, jj = i + di, j + dj
            if 0 <= ii < nx and 0 <= jj < ny and free[ii, jj] and w > 1e-3:
                acc += w * flow[ii, jj]; wsum += w
        if wsum == 0:
            break                              # no free corner: stop (InWall)
        x = x + h * acc / np.linalg.norm(acc)
        pts.append(x.copy())
        if x[0] < 0.2:
            break
    pts = np.array(pts)
    stop = pts[-1]
    ci, cj = int(np.floor(stop[0])), int(np.floor(stop[1]))
    ax.add_patch(Rectangle((ci, cj), 1, 1, color="C3", alpha=0.35, lw=0, zorder=1))
    for i in range(2, nx - 1):
        ax.add_patch(Rectangle((i, 2), 1, 1, color="C1", alpha=0.15, lw=0, zorder=1))
    # front: node to node, round the corner
    ax.plot([9, 2, 1, 1], [2, 2, 3, 6], "--", color="0.35", lw=1.6, zorder=3)
    ax.plot(pts[:, 0], pts[:, 1], "-", color="C3", lw=2.2, zorder=5)
    ax.plot(stop[0], stop[1], "x", color="C3", ms=10, mew=2.4, zorder=6)
    ax.annotate("vers le départ", (0.5, 6.35), (2.6, 6.35), color="0.25", fontsize=9,
                va="center", arrowprops=dict(arrowstyle="->", color="0.25"))
    ax.annotate("cellules mixtes : seuls les\ncoins libres comptent,\nrien ne repousse le chemin",
                (8.6, 2.6), (9.7, 1.0), fontsize=8.5, color="C1", ha="left", va="center",
                arrowprops=dict(arrowstyle="->", color="C1", lw=1.0))
    ax.annotate("aucun coin libre :\narrêt de l'extraction", (ci + 0.9, cj + 0.75), (9.7, 4.6),
                fontsize=8.5, color="C3", ha="left", va="center",
                arrowprops=dict(arrowstyle="->", color="C3", lw=1.0,
                                connectionstyle="arc3,rad=0.15"))
    ax.set_xlim(-0.6, nx + 3.9); ax.set_ylim(-0.6, ny - 0.2)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("(c) extraction, grille $100\\times100$", fontsize=11)
    return [Line2D([], [], ls="--", color="0.35", lw=1.6, label="front (de nœud en nœud)"),
            Line2D([], [], color="C3", lw=2.2, label="chemin extrait"),
            Line2D([], [], marker=r"$\rightarrow$", ls="", color="0.45", ms=10,
                   label="direction en un nœud libre"),
            Line2D([], [], marker="o", ls="", mfc="white", mec="k", label="nœud libre"),
            Line2D([], [], marker="o", ls="", color="k", label="nœud dans l'obstacle")]


def main():
    fig = plt.figure(figsize=(16, 4.6))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.05], wspace=0.10)
    ax_a, ax_b = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    schema(ax_a, 0.125, 0.0, "(a) case de 12,5 cm, alignée sur les parois")
    schema(ax_b, 0.20, 0.10, "(b) case de 20 cm, décalée")
    ax_b.legend(handles=[Patch(facecolor="white", edgecolor="k", hatch="////", label="obstacle réel"),
                         Patch(color=GREY, label="case marquée obstacle"),
                         Line2D([], [], color="C0", lw=2, label="chariot"),
                         Patch(facecolor="C0", alpha=0.25, edgecolor="C0", label="cercles de recouvrement")],
                loc="upper center", bbox_to_anchor=(0.0, -0.02), ncol=4, fontsize=9, frameon=False)
    ax_c = fig.add_subplot(gs[2])
    h = schema_c(ax_c)
    ax_c.legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=2,
                fontsize=8.5, frameon=False)
    os.makedirs(S.FIGURES, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(S.FIGURES, "np_rasterisation.%s" % ext), dpi=150, bbox_inches="tight")
    plt.close(fig)
    S.log("  circles of radius %.3f m, envelope %.3f m" % (R, 2 * R))


if __name__ == "__main__":
    main()
