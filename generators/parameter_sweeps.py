"""Paths of the one-parameter sweeps and of the k2 sweep, and the situations.

Reads ``data/sensitivity/one_parameter`` (``scripts/acquire_sensitivity.py
--part one_parameter``): Two-Rooms at 200 cells a side, one parameter moved at
a time around the centre (k1 = 10, k2 = 50, nu_FWD = 3, nu_2WS = 1.5,
nu_4WS = 0.5, gamma_rev = 0.5); and ``data/sensitivity/k1_k2_map`` for k2 at
k1 = 2 on the docking and Narrow-Passage problems.

The map is drawn in the orientation of Bench-MR and of the comparisons (rooms
on the side): a quarter turn clockwise, ``(x, y) -> (y, H - x)``, heading
``theta - pi/2``. Each path has its start closed on the requested pose
(:func:`shfm.splice.close_start`); a path whose end lies further than 0.75 m
from the start pose, or whose reach error exceeds 0.5 m, is left out.

Writes ``sens_situations`` (start and goal poses of the four situations) and
one panel per sweep, ``sens_<parameter>_<situation>``. CPU only.
"""
import csv
import os

import numpy as np
import matplotlib.colors as mcolors
from matplotlib import cm
from matplotlib.patches import Polygon

import study_tools as S
from study_tools import plt
from shfm import DEFAULT_VEHICLE
from shfm.environments import from_occupancy
from shfm.postprocess import dechatter
from shfm.splice import close_start

SWEEP = os.path.join(S.DATA, "sensitivity", "one_parameter")

SCALE = 2.0
"""The poses are defined on the hundred-cell grid; the sweeps run at 200."""

POSES_100 = {
    "S1": dict(seed=[30, 70, np.pi / 2, 0], tip=[90, 80, np.pi, 0]),
    "S5": dict(seed=[20, 28, 0, 0], tip=[80, 28, 0, 0]),
    "R1": dict(seed=[20, 22, 0, 0], tip=[80, 22, np.pi, 0]),
    "R2": dict(seed=[28, 76, np.pi / 2, 0], tip=[80, 22, 0, 0]),
    "R3": dict(seed=[20, 22, 0, 0], tip=[82, 80, 3 * np.pi / 2, 0]),
}
"""As ``SCENARIOS`` in ``scripts/acquire_sensitivity.py``."""
POSES = {t: dict(seed=[SCALE * p["seed"][0], SCALE * p["seed"][1]] + list(p["seed"][2:]),
                 tip=[SCALE * p["tip"][0], SCALE * p["tip"][1]] + list(p["tip"][2:]))
         for t, p in POSES_100.items()}

SIT = {"traversee": "S1", "demitour": "R1", "sortie": "R2", "entree": "R3"}
NOM = {"traversee": "Traversée", "demitour": "Demi-tour", "sortie": "Sortie", "entree": "Entrée"}

TARGET_VALUES = {
    "k1": [2, 6, 10, 20, 30, 40, 48],
    "k2": [12, 20, 30, 50, 100, 150, 200],
    "nu_FWD": [1.6, 2, 3, 5, 7, 10],
    "nu_2WS": [0.6, 1, 1.5, 2, 2.5, 2.9],
    "nu_4WS": [0.1, 0.25, 0.5, 0.75, 1, 1.4],
    "nu_reverse": [0, 0.25, 0.5, 0.6, 0.75, 1],
}
"""Values drawn per parameter: the nearest valid solve to each."""
REFERENCE = {"k1": 10.0, "k2": 50.0, "nu_FWD": 3.0, "nu_2WS": 1.5,
             "nu_4WS": 0.5, "nu_reverse": 0.5}
LABELS = {"k1": "$k_1$", "k2": "$k_2$", "nu_reverse": r"$\gamma_{\mathrm{rev}}$",
          "nu_FWD": r"$\nu_{\mathrm{FWD}}$", "nu_2WS": r"$\nu_{\mathrm{2WS}}$",
          "nu_4WS": r"$\nu_{\mathrm{4WS}}$"}

PANELS = [("k1", "demitour"), ("k1", "entree"),
          ("nu_reverse", "demitour"), ("nu_reverse", "sortie"), ("nu_reverse", "entree"),
          ("nu_reverse", "traversee"),
          ("nu_FWD", "traversee"), ("nu_2WS", "sortie"), ("nu_4WS", "traversee"),
          ("nu_4WS", "demitour")]
YLAB = {("k1", "demitour"), ("nu_reverse", "demitour"), ("nu_FWD", "traversee")}
"""Left panel of each row of the printed figures: the only ones with a y axis."""

FT, FL, FCB, FCBL, LWK = 17, 19, 16, 18, 1.35
"""Font sizes of ticks, axis labels, colour-bar ticks and title; line width factor."""

REAR_AXLE = 0.4845
"""Rear axle to rear end of the vehicle, in metres."""

_env = np.load(os.path.join(SWEEP, "env_two_rooms.npz"), allow_pickle=True)
WALLS, X0, X1 = _env["walls"], _env["X0"], _env["X1"]
ZOOM = int(_env["zoom"])
RLEN, RWID = float(_env["rlength"]), float(_env["rwidth"])
GEOS = np.load(os.path.join(SWEEP, "geos_1d_two_rooms.npz"))
H_ROT = float(X0.min() + X0.max())
SPLICE_ENV = {}


def fmt(v):
    if abs(v - round(v)) < 1e-9:
        return "%d" % int(round(v))
    return ("%.2f" % v).rstrip("0").rstrip(".")


def T(x, y, th=None):
    """To the orientation of Bench-MR: a quarter turn clockwise."""
    return (y, H_ROT - x) if th is None else (y, H_ROT - x, th - np.pi / 2)


def load_swept(scen):
    """``(param, value) -> (swept area, reach error)`` of one situation."""
    lut = {}
    with open(os.path.join(SWEEP, "swept_1d_two_rooms.csv")) as f:
        for r in csv.DictReader(f):
            if r["scenario"] != scen:
                continue
            try:
                sw, re = float(r["swept_area_m2"]), float(r["reach_error"])
            except (ValueError, KeyError):
                sw, re = float("nan"), float("nan")
            lut[(r["param"], float(r["value"]))] = (sw, re)
    return lut


def select(swept, scen, param, seed, dropped):
    """The solves drawn for one sweep: for each target value, the nearest
    value whose path is valid (reach error at most 2 cells per 100, end within
    3 cells per 100 of the start)."""
    pref = "%s__%s__" % (scen, param)
    good = []
    for k in GEOS.files:
        if not k.startswith(pref):
            continue
        v = float(k[len(pref):])
        sw, re = swept.get((param, v), (float("nan"), float("nan")))
        if not (np.isfinite(re) and re <= 2.0 * SCALE):
            continue
        g = GEOS[k]
        if g.ndim != 2 or min(g.shape) < 2:
            continue
        d = float(np.linalg.norm(g[:2, -1] - seed))
        if d > 3.0 * SCALE:
            dropped.append((param, v, d))
            continue
        good.append((v, k, sw, re))
    good.sort(key=lambda t: t[0])
    vals = np.array([e[0] for e in good])
    chosen, used = [], set()
    for t in TARGET_VALUES[param]:
        if not len(vals):
            break
        i = int(np.argmin(np.abs(vals - t)))
        if i in used:
            continue
        used.add(i)
        chosen.append(good[i])
    return chosen


def two_hundred_square():
    """The Two-Rooms scene of the sweeps, lifted on a hundred orientations
    (collision test of the splice)."""
    from PIL import Image
    img = np.array(Image.open(os.path.join(S.MAPS, "two_rooms.png")).convert("L")) < 128
    img = np.kron(img, np.ones((2, 2), dtype=bool))
    return from_occupancy(img, zoom=8.0, n_theta=100, vehicle=DEFAULT_VEHICLE)


def spliced(g, scen):
    """Geodesic as the solver returns it (goal to start), its start closed on
    the requested pose."""
    if g.shape[0] < 4:
        return g
    if "env" not in SPLICE_ENV:
        SPLICE_ENV["env"] = two_hundred_square()
    env = SPLICE_ENV["env"]
    q, _info = close_start(g.T[::-1], POSES[scen]["seed"][:3], env,
                           DEFAULT_VEHICLE.wheelbase * env.zoom, 0.5, 0.5)
    return q[::-1].T


def robot_corners(bot_dim, zoom):
    """Corners of the vehicle outline, rear axle at the origin, front towards -x."""
    Lf = bot_dim[0] * zoom - REAR_AXLE * zoom
    Lb = REAR_AXLE * zoom
    w = 0.5 * bot_dim[1] * zoom
    return np.array([[-Lf, -w], [-Lf, w], [Lb, w], [Lb, -w]])


def footprint(ax, x, y, th, zoom, bot_dim, color, nose_inside=False):
    """Vehicle outline at the pose ``(x, y, th)`` and a nose giving the heading.

    ``nose_inside`` draws the nose within the outline: at the docking place,
    which is the size of the vehicle, a nose outside would overlap the obstacle.
    """
    corners = robot_corners(bot_dim, zoom)
    Lf = bot_dim[0] * zoom - REAR_AXLE * zoom
    w = 0.5 * bot_dim[1] * zoom
    if nose_inside:
        nose = np.array([[-Lf + 1.6 * w, -w], [-Lf + 1.6 * w, w], [-Lf, 0.0]])
    else:
        nose = np.array([[-Lf, -w], [-Lf, w], [-Lf - 1.6 * w, 0.0]])
    thd = th + np.pi
    c, s = np.cos(thd), np.sin(thd)
    R = np.array([[c, s], [-s, c]])
    ax.add_patch(Polygon(corners @ R + [x, y], closed=True, fill=True,
                         facecolor=color, alpha=0.30, edgecolor=color,
                         lw=2.2, zorder=6))
    ax.add_patch(Polygon(nose @ R + [x, y], closed=True, fill=True,
                         facecolor=color, edgecolor=color,
                         lw=1.0, zorder=7))


def base_map(ax, ylabel=True):
    """Two-Rooms turned to the orientation of Bench-MR, axes in cells."""
    ax.set_aspect("equal")
    ax.pcolormesh(X1, H_ROT - X0, WALLS.astype(float), cmap="Greys", vmin=0.0, vmax=1.7,
                  alpha=0.55, shading="auto")
    ax.set_xlim(X1.min(), X1.max()); ax.set_ylim(X0.min(), X0.max())
    t = np.arange(0, 25 * ZOOM + 1e-9, 50)
    ax.set_xticks(t); ax.set_yticks(t)
    ax.set_xticklabels(["%d" % v for v in t])
    ax.set_yticklabels(["%d" % v for v in t] if ylabel else [])
    ax.set_xlabel("x (cases)", fontsize=FL, labelpad=1)
    if ylabel:
        ax.set_ylabel("y (cases)", fontsize=FL, labelpad=1)
    ax.tick_params(labelsize=FT, pad=2)
    ax.grid(True, alpha=0.25)


def colorbar(fig, ax, cols, labels, title):
    n = len(cols)
    cmap = mcolors.ListedColormap(cols)
    norm = mcolors.BoundaryNorm(np.arange(n + 1) - 0.5, n)
    cb = fig.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, orientation="horizontal",
                      fraction=0.05, pad=0.13, aspect=30, ticks=np.arange(n))
    cb.ax.set_xticklabels(labels, fontsize=FCB)
    cb.set_label(title, fontsize=FCBL, labelpad=2)


def save_panel(fig, name):
    fig.subplots_adjust(left=0.14, right=0.98, top=0.98, bottom=0.08)
    os.makedirs(S.FIGURES, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(S.FIGURES, name + "." + ext), dpi=120, bbox_inches="tight")
    plt.close(fig)


def sweep_panel(param, sit):
    """Paths of one sweep in one situation, coloured by the parameter value."""
    scen = SIT[sit]
    fig, ax = plt.subplots(figsize=(4.6, 5.0))
    swept = load_swept(scen)
    seed = np.array(POSES_100[scen]["seed"][:2], dtype=float) * SCALE
    dropped = []
    sel = select(swept, scen, param, seed, dropped)
    base_map(ax, ylabel=(param, sit) in YLAB)
    n = len(sel)
    cols = [plt.get_cmap("viridis")(i / max(1, n - 1)) for i in range(n)]
    for i, (v, k, sw, re) in enumerate(sel):
        g = spliced(GEOS[k], scen)
        lw = LWK * (3.2 - 1.8 * i / max(1, n - 1))
        ax.plot(*T(g[0], g[1]), color=cols[i], lw=lw, solid_capstyle="round", zorder=3 + i)
    p = POSES[scen]
    footprint(ax, *T(*p["seed"][:3]), ZOOM, (RLEN, RWID), "tab:blue")
    footprint(ax, *T(*p["tip"][:3]), ZOOM, (RLEN, RWID), "tab:red")
    colorbar(fig, ax, cols, [fmt(v).replace(".", ",") + ("*" if abs(v - REFERENCE[param]) < 1e-6 else "")
                             for v, _, _, _ in sel], LABELS[param] + "  (* : référence)")
    name = "sens_%s_%s" % (param.replace("nu_reverse", "gamma_rev"), sit)
    save_panel(fig, name)
    S.log("  %s: %s; left out %s" % (name, [fmt(v) for v, *_ in sel],
                                     [(fmt(v), round(d)) for _, v, d in dropped]))


def k2_panel(scene, k1=2.0):
    """k2 at fixed k1 on a comparison problem (docking or Narrow-Passage)."""
    rows, paths = S.load_part("sensitivity", "k1_k2_map")
    sub = sorted([r for r in rows if r["scene"] == scene and abs(float(r["k1"]) - k1) < 1e-9
                  and r["key"] in paths], key=lambda r: float(r["k2"]))
    env = S.scene_env(scene)
    full = S.env_full(scene, 0)
    seed, tip = S.endpoints(scene)
    dl = S.CAMPAIGN_SCENES[scene][1]
    w = np.asarray(env.walls, dtype=float)
    fig, ax = plt.subplots(figsize=(4.6, 5.0))
    ax.imshow(np.where(w.T > 0, 0.72, 1.0), origin="lower", cmap="gray", vmin=0, vmax=1,
              extent=(-0.5, w.shape[0] - 0.5, -0.5, w.shape[1] - 0.5), interpolation="nearest")
    n = len(sub)
    cols = [plt.get_cmap("viridis")(i / max(1, n - 1)) for i in range(n)]
    for i, r in enumerate(sub):
        raw = dechatter(np.asarray(paths[r["key"]], dtype=float))[0]
        p, _ = close_start(raw, seed, full, DEFAULT_VEHICLE.wheelbase * full.zoom, dl, dl)
        lw = LWK * (3.2 - 1.8 * i / max(1, n - 1))
        ax.plot(p[:, 0], p[:, 1], color=cols[i], lw=lw, solid_capstyle="round", zorder=3 + i)
    z = env.zoom
    inside = scene == "docking"
    footprint(ax, seed[0], seed[1], seed[2], z, (RLEN, RWID), "tab:blue", nose_inside=inside)
    footprint(ax, tip[0], tip[1], tip[2], z, (RLEN, RWID), "tab:red", nose_inside=inside)
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, w.shape[0] - 0.5); ax.set_ylim(-0.5, w.shape[1] - 0.5)
    step = 20 if scene == "docking" else 50
    t = np.arange(0, w.shape[0] + 1e-9, step)
    ax.set_xticks(t); ax.set_xticklabels(["%d" % v for v in t])
    t = np.arange(0, w.shape[1] + 1e-9, step)
    ax.set_yticks(t); ax.set_yticklabels(["%d" % v for v in t])
    ax.set_xlabel("x (cases)", fontsize=FL, labelpad=1); ax.set_ylabel("y (cases)", fontsize=FL, labelpad=1)
    ax.tick_params(labelsize=FT, pad=2); ax.grid(True, alpha=0.25)
    colorbar(fig, ax, cols, ["%g" % float(r["k2"]) for r in sub], "$k_2$  ($k_1 = %g$)" % k1)
    save_panel(fig, "sens_k2_%s" % ("accostage" if scene == "docking" else "np"))


LABEL_SITU = {("sortie", "d"): (-2.0, -3.0, "right", "top"),
              ("traversee", "d"): (0.0, 4.0, "left", "bottom")}
"""Placement of some labels of the situations (offset in cells / SCALE, alignment)."""


def situations():
    """Start (disc) and goal (square) poses, arrow along the heading, name of
    the situation beside each pose; two panels."""
    fig, axs = plt.subplots(1, 2, figsize=(9.2, 4.9))
    groups = ((axs[0], ("traversee", "sortie"), "(a) Traversée et Sortie"),
              (axs[1], ("demitour", "entree"), "(b) Demi-tour et Entrée"))
    for ax, keys, ttl in groups:
        ax.set_aspect("equal")
        ax.pcolormesh(X1, H_ROT - X0, WALLS.astype(float), cmap="Greys", vmin=0.0, vmax=1.7,
                      alpha=0.55, shading="auto")
        ax.set_xlim(X1.min(), X1.max()); ax.set_ylim(X0.min(), X0.max())
        t = np.arange(0, 25 * ZOOM + 1e-9, 50)
        ax.set_xticks(t); ax.set_yticks(t)
        ax.set_yticklabels(["%d" % v for v in t] if ax is axs[0] else [])
        ax.set_xlabel("x (cases)", fontsize=10)
        if ax is axs[0]:
            ax.set_ylabel("y (cases)", fontsize=10)
        ax.tick_params(labelsize=9); ax.grid(True, alpha=0.25)
        spots = {}
        for key in keys:
            sc = POSES[SIT[key]]
            for q, kind in ((sc["seed"], "d"), (sc["tip"], "a")):
                X, Y, TH = T(*q[:3])
                spots.setdefault((round(X), round(Y)), []).append((key, kind, TH))
        for (x, y), items in spots.items():
            for key, kind, th in items:
                ax.annotate("", (x + 8 * SCALE * np.cos(th), y + 8 * SCALE * np.sin(th)), (x, y),
                            arrowprops=dict(arrowstyle="-|>", color="0.25", lw=1.6), zorder=4)
            if any(kind == "a" for _, kind, _ in items):
                ax.plot(x, y, "s", ms=10, mfc="white", mec="k", mew=1.4, zorder=5)
            if any(kind == "d" for _, kind, _ in items):
                ax.plot(x, y, "o", ms=6, color="k", zorder=6)
            txt = "\n".join("%s : %s" % (NOM[k], "départ" if kind == "d" else "arrivée")
                            for k, kind, _ in items)
            right = x > 70 * SCALE
            tx, ty = x + (-2.5 if right else 2.5) * SCALE, y + (3.0 if y < 50 * SCALE else -3.5) * SCALE
            ha, va = ("right" if right else "left"), ("bottom" if y < 50 * SCALE else "top")
            for (k, kind), (dx, dy, h, v) in LABEL_SITU.items():
                if [(kk, kd) for kk, kd, _ in items] == [(k, kind)]:
                    tx, ty, ha, va = x + dx * SCALE, y + dy * SCALE, h, v
            ax.text(tx, ty, txt, fontsize=8, ha=ha, va=va, zorder=7,
                    bbox=dict(fc="white", ec="none", alpha=0.85, pad=0.6))
        ax.set_title(ttl, fontsize=11)
    fig.subplots_adjust(left=0.07, right=0.99, top=0.93, bottom=0.1, wspace=0.06)
    os.makedirs(S.FIGURES, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(S.FIGURES, "sens_situations." + ext), dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    situations()
    for scene in ("docking", "narrow_passage"):
        k2_panel(scene)
    for param, sit in PANELS:
        sweep_panel(param, sit)


if __name__ == "__main__":
    main()
