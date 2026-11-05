"""Shared helpers of the figure scripts of the parameter study and of the
comparison with the sampling planners.

They read the acquisitions shipped under ``data/`` (written by
``scripts/acquire_sensitivity.py`` and ``scripts/acquire_comparison.py``),
measure our paths with the Bench-MR metrics, and draw. CPU only.

``SHFM_DATA_DIR`` points the scripts at another data directory (for instance
``runs/`` after a rebuild), ``SHFM_FIGURE_DIR`` at another output directory; a
relative path is taken from the current directory. The map images are always
read from ``data/maps/`` of the repository.
"""
import csv
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import matplotlib                                                   # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                     # noqa: E402
from matplotlib.collections import LineCollection                   # noqa: E402

from shfm import DEFAULT_VEHICLE                                    # noqa: E402
from shfm import benchmark as B, metrics as M                       # noqa: E402
from shfm.environments import from_benchmark_run                    # noqa: E402
from shfm.postprocess import chatter_runs, dechatter, reversals     # noqa: E402
from shfm.splice import close_start                                 # noqa: E402
from shfm.states import SOLVER_ORDER                                # noqa: E402

DATA = os.path.abspath(os.environ.get("SHFM_DATA_DIR") or os.path.join(REPO, "data"))
FIGURES = os.path.abspath(os.environ.get("SHFM_FIGURE_DIR") or os.path.join(REPO, "figures"))
MAPS = os.path.join(REPO, "data", "maps")
"""The map images ship with the repository; the acquisitions do not rewrite them."""
TABLES = os.path.join(FIGURES, "tables")

CAMPAIGN_SCENES = {
    # name             zoom  steering saturation (rad)
    "two_rooms":      (8.0, 0.5),
    "warehouse":      (2.0, 0.5),
    "procedural":     (2.0, 0.5),
    "docking":        (20.0, 1.05),
    "narrow_passage": (8.0, 0.5),
}
"""As in ``scripts/acquisition.py``."""
ZOOM = {k: v[0] for k, v in CAMPAIGN_SCENES.items()}
NAMES = {"two_rooms": "Two-Rooms", "warehouse": "Warehouse",
         "procedural": "Procédural", "docking": "Accostage",
         "narrow_passage": "Narrow-Passage"}
MODE_COLOR = {"nav_fwd": "#1f77b4", "nav_bwd": "#ff7f0e",
              "man_fwd": "#2ca02c", "man_bwd": "#d62728"}
MODE_FR = {"nav_fwd": "navigation avant", "nav_bwd": "navigation arrière",
           "man_fwd": "manœuvre avant", "man_bwd": "manœuvre arrière"}
METHOD_COLOR = {"2WS": (0, 0, 1, 1), "4WS": (0.85, 0, 0, 1)}


def log(*a):
    print(*a, flush=True)


def bench_python():
    """Put the plotting modules of Bench-MR on the import path, once its metric
    bindings are loaded (they import them)."""
    M.bindings()
    path = os.path.join(REPO, "external", "bench-mr", "python")
    if path not in sys.path:
        sys.path.insert(0, path)


def save(fig, name, directory=None):
    """``name.png`` and ``name.pdf`` in the figure directory."""
    d = directory or FIGURES
    os.makedirs(d, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(d, "%s.%s" % (name, ext)), dpi=150,
                    bbox_inches="tight")
    plt.close(fig)


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load_part(*part):
    """``(rows, paths)`` of one acquisition part, e.g. ``("comparison",
    "campaigns")``, or ``(None, None)`` when it is missing."""
    d = os.path.join(DATA, *part)
    if not os.path.exists(os.path.join(d, "rows.csv")):
        return None, None
    rows = list(csv.DictReader(open(os.path.join(d, "rows.csv"))))
    npz = os.path.join(d, "paths.npz")
    paths = dict(np.load(npz).items()) if os.path.exists(npz) else {}
    return rows, paths


def write_table(name, text):
    """A table of values, written under ``<figure directory>/tables/``."""
    os.makedirs(TABLES, exist_ok=True)
    with open(os.path.join(TABLES, name), "w") as f:
        f.write(text)
    log("  wrote tables/%s" % name)


def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |",
           "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out) + "\n"


def frac_man(p):
    """Share of the points of a path in a manoeuvring mode."""
    m = np.rint(np.asarray(p)[:, 3]).astype(int)
    return float(np.mean((m == 1) | (m == 2)))


def total_turn(p):
    """Sum of the heading changes along a path, in radians."""
    th = np.unwrap(np.asarray(p, dtype=float)[:, 2])
    return float(np.abs(np.diff(th)).sum())


# ------------------------------------------------------------ scenes

_WALLS = {}


def scene_env(scene, run=0):
    """Map of one campaign run, lifted on two orientations only (walls are enough)."""
    key = (scene, run)
    if key not in _WALLS:
        r = B.load_campaign(scene)["runs"][run]
        _WALLS[key] = from_benchmark_run(r, zoom=ZOOM[scene], n_theta=2,
                                         vehicle=DEFAULT_VEHICLE)
    return _WALLS[key]


def endpoints(scene, run=0):
    s, g = B.endpoints_of(B.load_campaign(scene)["runs"][run])
    return list(s), list(g)


_ENV_FULL = {}


def env_full(scene, run):
    """Campaign map lifted on the hundred orientations of the solve (collision
    test of :func:`shfm.splice.close_start`)."""
    if (scene, run) not in _ENV_FULL:
        _ENV_FULL.clear()
        r = B.load_campaign(scene)["runs"][run]
        _ENV_FULL[(scene, run)] = from_benchmark_run(r, zoom=ZOOM[scene], n_theta=100,
                                                     vehicle=DEFAULT_VEHICLE)
    return _ENV_FULL[(scene, run)]


def clean_path(p, scene, run, method):
    """Extraction chatter removed, then the start closed on the requested pose."""
    q = dechatter(p)[0]
    _, delta = CAMPAIGN_SCENES[scene]
    return close_start(q, endpoints(scene, run)[0], env_full(scene, run),
                       DEFAULT_VEHICLE.wheelbase * ZOOM[scene], delta,
                       delta if method == "4WS" else 0.0)


# ------------------------------------------------------------ measures

def measure(path, env, goal):
    """Bench-MR metrics of a driven path, curvature on the reversed (raw) order."""
    p = np.asarray(path, dtype=float)
    with M.quiet():
        mx, norm = M.curvature(p[::-1, :3])
        clear = M.clearing_distances(p[:, :3], env.walls)
        out = dict(path_length=M.path_length(p[:, :3]), aol=M.aol(p[:, :3]),
                   max_curvature=mx, normalized_curvature=norm,
                   min_clearing=float(clear.min()), mean_clearing=float(clear.mean()),
                   swept_area_m2=M.swept_area(p[:, :3], env.vehicle, env.zoom))
    out["n_cusps"] = len(reversals(p))
    return out


def shipped_reference(scene):
    """Mean length, clearings and swept area of RRT* and Informed-RRT*, over
    the runs of the campaign that reached the goal."""
    data = B.load_campaign(scene)
    out = {}
    for run in data["runs"]:
        for name, plan in run["plans"].items():
            if name not in ("RRTstar", "InformedRRTstar") or not plan or not B.reached_goal(plan):
                continue
            st = plan["stats"]
            o = out.setdefault(name, {"length": [], "mean_c": [], "min_c": []})
            o["length"].append(st["path_length"])
            o["mean_c"].append(st["mean_clearing_distance"])
            o["min_c"].append(st["min_clearing_distance"])
    sw = {}
    for r in csv.DictReader(open(os.path.join(DATA, "sensitivity", "benchmr_swept.csv"))):
        if r["json"] == scene and r["planner"] in ("RRTstar", "InformedRRTstar"):
            sw.setdefault(r["planner"], []).append(fnum(r["swept_area_m2"]))
    res = {}
    for name, o in out.items():
        res[name] = {k: float(np.mean(v)) for k, v in o.items()}
        res[name]["swept"] = float(np.mean(sw.get(name, [np.nan])))
    return res


# ------------------------------------------------------------ drawing

def draw_map(ax, walls, zoom):
    w = np.asarray(walls, dtype=float)
    ax.imshow(np.where(w.T > 0, 0.72, 1.0), origin="lower", cmap="gray",
              vmin=0, vmax=1, extent=(-0.5, w.shape[0] - 0.5, -0.5, w.shape[1] - 0.5),
              interpolation="nearest")
    ax.set_xlabel("$x$ (units ; 1 unit = %.3g m)" % (1 / zoom))
    ax.set_ylabel("$y$ (units ; 1 unit = %.3g m)" % (1 / zoom))
    ax.set_aspect("equal")


def draw_modes(ax, path, lw=2.0, cusps=True, chatter=True):
    """A path coloured by mode, cusps marked with crosses, chatter circled."""
    p = np.asarray(path, dtype=float)
    modes = np.clip(np.rint(p[:, 3]).astype(int), 0, 3)
    seg = np.stack([p[:-1, :2], p[1:, :2]], axis=1)
    cols = [MODE_COLOR[SOLVER_ORDER[m]] for m in modes[:-1]]
    ax.add_collection(LineCollection(seg, colors=cols, linewidths=lw))
    if chatter:
        for a, b in chatter_runs(p):
            q = p[a:b + 2]
            ax.plot(q[:, 0].mean(), q[:, 1].mean(), "o", ms=11, mfc="none",
                    mec="magenta", mew=2)
    if cusps:
        f = reversals(p)
        ch = set()
        for a, b in chatter_runs(p):
            ch.update(range(a, b + 1))
        real = [i for i in f if i not in ch]
        if real:
            ax.plot(p[np.array(real) + 1, 0], p[np.array(real) + 1, 1], "kx",
                    ms=8, mew=2)
