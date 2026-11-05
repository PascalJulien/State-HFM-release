"""Grid-resolution study: cost, convergence of the length, and the paths.

Reads ``data/sensitivity/resolution`` (``scripts/acquire_sensitivity.py
--part resolution`` and ``--part memory``): three problems, Two-Rooms,
Narrow-Passage and Warehouse, each solved on six grids from 50 to 200 cells a
side, the map cut into cells with the conservative rasterisation.

Writes:

* ``resolution_cost_convergence``: per scene, the solve time and the peak GPU
  memory against the number of grid points, with the exponent of a power-law
  fit of the time, and the path length relative to the finest grid;
* ``resolution_traj_overlay``: the paths of the six grids over each map,
  coarse (dark) to fine (light); a path that ends further than a vehicle
  length from its start or goal is dashed.

CPU only.
"""
import csv
import json
import os

import numpy as np
import matplotlib.cm as cm
from matplotlib.patches import Polygon

import study_tools as S
from study_tools import plt

DATA = os.path.join(S.DATA, "sensitivity", "resolution")
SCEN = [("two_rooms", "Two-Rooms"), ("narrow_passage", "Narrow-Passage"), ("warehouse", "Warehouse")]
REAR_AXLE = (0.4845, 0.0)
"""Rear axle relative to the rear end of the vehicle, in metres."""


def cost_convergence():
    rows = list(csv.DictReader(open(os.path.join(DATA, "sweep_resolution.csv"))))
    # Peak memory in use, measured in fresh processes: in the sweep
    # the memory held by the cupy pool depended on the previous solves.
    mem = {}
    for line in open(os.path.join(DATA, "memory.jsonl")):
        e = json.loads(line)
        mem[(e["scene"], e["n"])] = e["peak_used_mb"]
    t_ticks = np.arange(0, 6.01, 1.0)          # s
    x_lim = (0, 4.2e6)
    conv = {"two_rooms": (0.85, 1.11), "narrow_passage": (0.85, 3.97), "warehouse": (0.85, 1.11)}

    fig, axes = plt.subplots(3, 2, figsize=(11, 12))
    for i, (key, name) in enumerate(SCEN):
        s = [r for r in rows if r["scenario"] == key]
        N = np.array([float(r["npoints"]) for r in s])
        t = np.array([float(r["solve_s"]) for r in s])
        g = np.array([mem.get((key, int(r["Nx"])), float(r["gpu_mb"])) for r in s]) / 1e3
        ok = np.array([r["degenerate"] != "1" for r in s])
        a, _ = np.polyfit(np.log(N), np.log(t), 1)
        ax = axes[i, 0]
        l1, = ax.plot(N, t, "o-", color="C0", label="temps de solve (s)")
        l2, = ax.plot(N, g, "s--", color="C3", label="pic mémoire GPU (Go)")
        ax.set_xlim(*x_lim); ax.set_ylim(t_ticks[0], t_ticks[-1]); ax.set_yticks(t_ticks)
        ax.set_xlabel("points de grille $N_x\\,N_y\\,N_\\theta$")
        ax.set_ylabel("temps (s) ; mémoire (Go)")
        ax.grid(True, alpha=.3)
        ax.set_title("%s, coût ($t \\propto N^{%.2f}$)" % (name, a))
        ax.legend(handles=[l1, l2], loc="upper left", fontsize=8)
        ax = axes[i, 1]
        L = np.array([float(r["path_length"]) if r["path_length"] not in ("", "nan") else np.nan
                      for r in s])
        ax.plot(N[ok], L[ok] / L[-1], "o-", label="longueur")
        ax.axhspan(0.95, 1.05, color="grey", alpha=.12, zorder=0)
        ax.axhline(1.0, color="k", lw=.6, alpha=.5)
        ax.set_xscale("log"); ax.set_xlim(1.6e5, 4.6e6); ax.set_ylim(*conv[key])
        ax.set_xlabel("points de grille $N_x\\,N_y\\,N_\\theta$")
        ax.set_ylabel("longueur / longueur à résolution max")
        ax.set_title("%s, convergence de la longueur" % name)
        ax.grid(True, which="both", alpha=.3)
    fig.tight_layout()
    os.makedirs(S.FIGURES, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(S.FIGURES, "resolution_cost_convergence.%s" % ext), dpi=140)
    plt.close(fig)


def pose_robot(ax, pose, zoom, color, rlength, rwidth):
    """Vehicle outline at a pose, rear axle on the point, and an arrow along
    the heading, one vehicle length long."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    Lf = rlength * zoom - REAR_AXLE[0] * zoom
    Lb = REAR_AXLE[0] * zoom
    w = 0.5 * rwidth * zoom
    corners = np.array([[-Lf, -w], [-Lf, w], [Lb, w], [Lb, -w]])     # front towards -x
    a = th + np.pi
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    pts = corners @ R.T + np.array([x, y])
    ax.add_patch(Polygon(pts, closed=True, facecolor=color, alpha=0.35,
                         edgecolor=color, lw=1.6, zorder=24))
    L = rlength * zoom
    ax.arrow(x, y, L * np.cos(th), L * np.sin(th), width=0.18 * zoom,
             head_width=0.9 * zoom, head_length=0.9 * zoom,
             length_includes_head=True, color=color, ec=color, lw=0.5,
             zorder=25, alpha=0.95)


def paths_overlay():
    d = np.load(os.path.join(DATA, "sweep_resolution_geos.npz"))
    rlength, rwidth = float(d["rlength"]), float(d["rwidth"])
    degenerate = {}
    for r in csv.DictReader(open(os.path.join(DATA, "sweep_resolution.csv"))):
        degenerate[(r["scenario"], "%sx%sx%s" % (r["Nx"], r["Ny"], r["Ntheta"]))] = \
            r.get("degenerate", "0") == "1"

    def res_keys(scen):
        """Path keys ``<scene>__NxxNyxNt``, sorted by number of grid points."""
        out = []
        pref = "%s__" % scen
        for k in d.files:
            if not k.startswith(pref):
                continue
            suf = k[len(pref):]
            parts = suf.split("x")
            if len(parts) != 3 or not all(p.isdigit() for p in parts):
                continue
            nx, ny, nt = map(int, parts)
            out.append((nx * ny * nt, suf, k))
        return sorted(out)

    def draw(ax, scen, title):
        walls = d["%s__walls" % scen].astype(float)
        X0, X1 = d["%s__X0" % scen], d["%s__X1" % scen]
        zoom = int(d["%s__zoom" % scen])
        ax.set_aspect("equal")
        ax.pcolormesh(X0, X1, walls, cmap="Greys", vmin=0.0, vmax=1.7,
                      alpha=0.55, shading="auto")
        keys = res_keys(scen)
        n = len(keys)
        colors = cm.viridis(np.linspace(0.05, 0.9, n))
        for i, (npts, suf, k) in enumerate(keys):
            g = d[k]
            lw = 1.3 + 1.6 * i / (n - 1)
            deg = degenerate.get((scen, suf), False)
            npt = "%.0f k" % (npts / 1e3) if npts < 1e6 else "%.2g M" % (npts / 1e6)
            ax.plot(g[:, 0], g[:, 1], color=colors[i],
                    lw=(lw * 0.7 if deg else lw), alpha=(0.45 if deg else 0.9),
                    ls=("--" if deg else "-"), solid_capstyle="round", zorder=3 + i,
                    label="%s  (%s)%s" % (suf, npt, "  dég." if deg else ""))
        pose_robot(ax, d["%s__seed" % scen], zoom, "blue", rlength, rwidth)
        pose_robot(ax, d["%s__tip" % scen], zoom, "red", rlength, rwidth)
        note = "1 unit = %.3g m" % (1.0 / float(zoom))
        ax.set_xlabel("$x$ (units ; %s)" % note, fontsize=18)
        ax.set_ylabel("$y$ (units ; %s)" % note, fontsize=18)
        ax.tick_params(labelsize=14)
        ax.set_xlim(X0.min(), X0.max()); ax.set_ylim(X1.min(), X1.max())
        ax.set_title(title, fontsize=16, pad=6)
        leg = ax.legend(loc="best", fontsize=8, framealpha=0.9,
                        title=r"$N_x{\times}N_y{\times}N_\theta$", title_fontsize=9)
        leg.set_zorder(30)

    scenes = [(k, v) for k, v in SCEN if "%s__walls" % k in d.files]
    fig, ax = plt.subplots(1, len(scenes), figsize=(6.5 * len(scenes), 6.4))
    for a, (scen, title) in zip(np.atleast_1d(ax), scenes):
        draw(a, scen, title)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(S.FIGURES, "resolution_traj_overlay.%s" % ext),
                    bbox_inches="tight", dpi=150)
    plt.close(fig)


def main():
    cost_convergence()
    paths_overlay()


if __name__ == "__main__":
    main()
