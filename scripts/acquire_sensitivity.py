#!/usr/bin/env python3
"""Acquisitions of the parameter study, shipped under ``data/sensitivity/``.

    python scripts/acquire_sensitivity.py --part envs             # seconds
    python scripts/acquire_sensitivity.py --part resolution       # about a minute
    python scripts/acquire_sensitivity.py --part memory           # a few minutes
    python scripts/acquire_sensitivity.py --part reference_swept  # a few minutes, CPU
    python scripts/acquire_sensitivity.py --part one_parameter    # about 15 minutes
    python scripts/acquire_sensitivity.py --part k1_k2_map        # about 10 minutes
    python scripts/acquire_sensitivity.py --part recommended      # under a minute
    python scripts/acquire_sensitivity.py --part clearance_cap    # about 15 minutes

Parts (each writes into ``<out>/<part>/``, except ``envs`` and
``reference_swept``, which write one file into ``<out>``, and ``memory``,
which writes into ``<out>/resolution/``):

* ``envs``: ``env_two_rooms.npz``, the Two-Rooms scene at a hundred cells a
  side that the navigation-function figure reads.
* ``resolution``: the same three problems solved on six grids, from 50 to 200
  cells a side, the map cut into cells with the conservative rasterisation;
  ``sweep_resolution.csv`` (time, memory, metrics) and
  ``sweep_resolution_geos.npz`` (the paths).
* ``memory``: the peak GPU memory of the same solves, each measured in a
  fresh process (``memory.jsonl``). In a single process the memory pool of
  cupy keeps blocks from the previous solves, so the memory read by
  ``resolution`` depends on the order of the solves.
* ``reference_swept``: ``benchmr_swept.csv``, the floor area swept by the
  trajectories the sampling planners returned in the campaigns of
  ``data/campaigns/``.
* ``one_parameter``: one parameter moved at a time around the centre
  ``BASE``, on five Two-Rooms situations at 200 cells a side (``SITUATIONS``),
  clearance capped at the vehicle length, start mode free.
* ``k1_k2_map``: a grid of transition costs ``1 <= k1 < k2`` on the same
  situations, and on the docking and Narrow-Passage problems of the campaigns.
* ``recommended``: the two settings (2, 3) and (10, 50), solved as the
  one-parameter sweeps are, on the same situations.
* ``clearance_cap``: the cap on the obstacle cost, from no
  obstacle cost to no cap, on the Two-Rooms, Warehouse and procedural
  campaign problems, at the transition costs (2, 3).

The output goes to ``runs/sensitivity/`` unless ``--out`` says otherwise, so
that a rebuild never overwrites the shipped files. The solve times and the
memory of ``resolution`` and ``memory`` belong to the machine that measured
them; to measure them, run those parts on an otherwise idle machine (CPU and
GPU share the power budget of a laptop).

Needs a CUDA device (except ``envs`` and ``reference_swept``) and the
Bench-MR metric bindings (``scripts/setup.sh``).
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "scripts"))

from shfm import DEFAULT_VEHICLE, Planner                           # noqa: E402
from shfm import benchmark as B                                     # noqa: E402
from shfm.environments import from_occupancy, from_benchmark_run    # noqa: E402
from shfm.states import transition_matrix                           # noqa: E402

import acquisition as Q                                             # noqa: E402

OUT = os.path.join(HERE, "runs", "sensitivity")
"""Where the rebuilt files go. Set from ``--out``."""
MAPS = os.path.join(HERE, "data", "maps")

PI = np.pi

SCENARIOS = {
    # Two-Rooms at a hundred cells a side (zoom 4). S1, crossing from one room
    # to the other; S5, along the corridor.
    "S1": dict(seed=[30, 70, PI / 2, 0], tip=[90, 80, PI, 0]),
    "S5": dict(seed=[20, 28, 0, 0], tip=[80, 28, 0, 0]),
    # Poses whose orientations impose a change of direction: R1, U-turn in the
    # corridor; R2, leave a room whose back wall the vehicle faces; R3, end in
    # the other room facing its door. The start of R2 sits 7 cells (1.75 m)
    # from the wall the vehicle faces.
    "R1": dict(seed=[20, 22, 0, 0], tip=[80, 22, PI, 0]),
    "R2": dict(seed=[28, 76, PI / 2, 0], tip=[80, 22, 0, 0]),
    "R3": dict(seed=[20, 22, 0, 0], tip=[82, 80, 3 * PI / 2, 0]),
}
"""Start and goal poses of the situations, on the hundred-cell grid."""

SITUATIONS = ("S1", "S5", "R1", "R2", "R3")

BASE = dict(k1=10.0, k2=50.0, nu_reverse=0.5, nu_FWD=3.0, nu_2WS=1.5,
            nu_4WS=0.5)
"""The centre every sweep departs from, one parameter at a time."""

EXACT_VALUES = {
    "k1": [2, 6, 10, 20, 30, 40, 48],
    "k2": [12, 20, 30, 50, 100, 150, 200],
    "nu_FWD": [1.6, 2, 3, 5, 7, 10],
    "nu_2WS": [0.6, 1, 1.5, 2, 2.5, 2.9],
    "nu_4WS": [0.1, 0.25, 0.5, 0.75, 1, 1.4],
    "nu_reverse": [round(0.05 * i, 2) for i in range(21)],
}
"""Values of the one-parameter sweeps, inside the admissible domain around
``BASE``, which each list contains."""

MAP_GRID = [(k1, k2) for k1 in (1.0, 2.0, 5.0, 10.0, 20.0)
            for k2 in (2.0, 3.0, 5.0, 10.0, 20.0, 50.0, 100.0) if k1 < k2]
"""The (k1, k2) grid of ``k1_k2_map``."""

MAP_COEFFICIENTS = dict(nu_fwd=3.0, nu_2ws=1.5, nu_4ws=0.5, gamma_rev=0.5)

PASTSEED_200 = dict(geodesic_PastSeed_delay=1024)
"""``acquisition.PASTSEED`` for ``SITUATIONS`` at 200 cells a side. With 128 or 256 steps, the
paths of the entry facing the door that back into the room stopped 5.5 cells
(0.69 m) short of the start; 1024 lets them finish (1.6 cells), and 4096 gives
the same path."""

SATURATION_CAPS = (
    # label         cap in metres (None: no cap; 0: no obstacle cost)
    ("no_cost",     0.0),
    ("0.25m",       0.25),
    ("0.5m",        0.5),
    ("L/4",         DEFAULT_VEHICLE.length / 4),
    ("L/2",         DEFAULT_VEHICLE.length / 2),
    ("L",           DEFAULT_VEHICLE.length),
    ("2L",          2 * DEFAULT_VEHICLE.length),
    ("unsaturated", None),
)
"""Caps of ``clearance_cap``; the comparisons cap the clearance at the
vehicle length."""

ROBUST = Q.ROBUST


def log(*a):
    print(*a, flush=True)


def transitions_of(k1, k2):
    """The transition matrix, built here rather than through ``shfm.states``.

    It is the matrix of ``shfm.states.transition_matrix``, without its check
    that ``1 <= k1 < k2``: the sweeps go outside that range.
    """
    return np.array([[0, k2, k2, k1], [k2, 0, 1, k2],
                     [k2, 1, 0, k2], [k1, k2, k2, 0]], dtype=float)


# --------------------------------------------------------------------- metrics

def swept_area_m2(geo, corners, metres_per_unit, shape, sc=4, step=2):
    """Floor area of the union of the footprints along a geodesic.

    Each footprint is a rectangle placed at ``(x, y)`` and turned by
    ``theta + pi``, because ``Vehicle.corners`` puts the front of the vehicle
    on negative ``x``; it is rasterised on a canvas ``sc`` times finer than the
    grid. This is a lighter rasteriser than the Bench-MR one behind
    ``shfm.metrics.swept_area``, used for the many solves of the sweeps.
    """
    from PIL import Image, ImageDraw
    g = np.asarray(geo)
    if g.ndim != 2 or g.shape[1] < 2:
        return float("nan")
    x, y, th = g[0], g[1], g[2] + np.pi
    img = Image.new("1", (int(shape[0] * sc), int(shape[1] * sc)), 0)
    draw = ImageDraw.Draw(img)
    for i in range(0, len(x), step):
        c, s = np.cos(th[i]), np.sin(th[i])
        poly = corners @ np.array([[c, s], [-s, c]]) + np.array([x[i], y[i]])
        draw.polygon([(float(px * sc), float(py * sc)) for px, py in poly],
                     fill=1)
    return float(np.array(img).sum() * (metres_per_unit / sc) ** 2)


def reach_error(geo, seed, tip):
    """How far the geodesic's ends sit from the two configurations asked for.

    Flags the settings that return a path joining neither: it is the larger of
    the two end-to-nearest-target distances, in grid units.
    """
    g = np.asarray(geo)
    if g.ndim != 2 or g.shape[1] < 1:
        return float("nan")
    ends = np.array([[g[0, 0], g[1, 0]], [g[0, -1], g[1, -1]]])
    targets = np.array([[seed[0], seed[1]], [tip[0], tip[1]]])
    d = np.sqrt(((ends[:, None, :] - targets[None, :, :]) ** 2).sum(-1))
    return float(d.min(axis=1).max())


def sweep_solve(env, seed, tip, params, saturate=False, transitions=None,
                free_start=False, extra=None):
    """One solve at one point of a sweep; returns the geodesic ``(4, n)``.

    ``params`` holds the six parameters of ``BASE``; ``fm2 = 1``.
    """
    planner = Planner(env, k1=params["k1"], k2=params["k2"],
                      nu_fwd=params["nu_FWD"], nu_2ws=params["nu_2WS"],
                      nu_4ws=params["nu_4WS"], gamma_rev=params["nu_reverse"],
                      obstacle_cost=True, saturate=saturate, fm2=1.0,
                      transitions=(transitions_of(params["k1"], params["k2"])
                                   if transitions is None else transitions))
    result = planner.solve(seed=seed, tips=[tip], free_start_mode=free_start,
                           **ROBUST, **(extra or {}))
    return np.asarray(result.geodesics[0])


# ------------------------------------------------------------------ the scenes

def hundred_square():
    """Two-Rooms at a hundred cells a side, the map read as it stands."""
    from PIL import Image
    img = np.array(Image.open(os.path.join(MAPS, "two_rooms.png")
                              ).convert("L")) < 128
    return from_occupancy(img, zoom=4.0, n_theta=100, vehicle=DEFAULT_VEHICLE)


def two_hundred_square():
    """``hundred_square`` at two hundred cells a side (zoom 8), each cell
    split into 2 x 2: the resolution of the campaigns."""
    from PIL import Image
    img = np.array(Image.open(os.path.join(MAPS, "two_rooms.png")).convert("L")) < 128
    img = np.kron(img, np.ones((2, 2), dtype=bool))
    return from_occupancy(img, zoom=8.0, n_theta=100, vehicle=DEFAULT_VEHICLE)


def situations_200():
    """``(tag, seed, tip)`` of ``SITUATIONS``, poses on the 200-cell grid."""
    out = []
    for tag in SITUATIONS:
        s = SCENARIOS[tag]
        out.append((tag, [2 * s["seed"][0], 2 * s["seed"][1]] + list(s["seed"][2:]),
                    [2 * s["tip"][0], 2 * s["tip"][1]] + list(s["tip"][2:])))
    return out


def write_env(path, env, extra=None):
    payload = dict(walls=np.asarray(env.walls), X0=np.asarray(env.X[0]),
                   X1=np.asarray(env.X[1]), zoom=env.zoom,
                   rlength=env.vehicle.length, rwidth=env.vehicle.width,
                   offset=np.asarray([env.vehicle.rear_axle_offset, 0.0]))
    payload.update(extra or {})
    np.savez_compressed(path, **payload)
    log("  wrote %s" % os.path.basename(path))


# ------------------------------------------------------------ resolution study

RESOLUTION_STEPS = (50, 75, 100, 125, 150, 200)

RESOLUTION_SCENES = {
    # name            source             zoom  frame (units)
    "two_rooms":      ("map:two_rooms",   4.0, (100.0, 100.0)),
    "narrow_passage": ("narrow_passage",  8.0, (200.0, 200.0)),
    "warehouse":      ("warehouse",       2.0, (200.0, 150.0)),
}
"""The three scenes of the resolution study.

Each one has a fixed frame, in grid units, and the study varies how many cells
that frame is cut into, so a length measured at one resolution is comparable
with the next. Two-Rooms is measured on the hundred-unit frame of the
situations, with the endpoints of S1, the other two on the frame and
endpoints of their campaign. Two-Rooms reads its map as it stands, where
Bench-MR transposes it.
"""

RESOLUTION_SOLVE = {
    # Two-Rooms and Warehouse take the centre of the sweeps, clearance
    # unsaturated. Narrow-Passage prices every mode switch at one unit
    # (k1 = k2 = 1), with the clearance saturated.
    "two_rooms": dict(saturate=False),
    "warehouse": dict(saturate=False),
    "narrow_passage": dict(saturate=True, transitions=1.0 - np.eye(4)),
}

RASTERIZE = "conservative"
"""How the resolution study cuts the map into coarser or finer cells.

``conservative`` marks a cell as an obstacle as soon as it overlaps one
obstacle cell of the source map, so a coarse grid can only close passages,
never open them. ``bilinear`` interpolates the occupancy and keeps a cell as
an obstacle when the interpolated value exceeds one half; on a coarse grid
that erases obstacles thinner than a cell and can open a passage the vehicle
cannot use. Set from ``--rasterize``.
"""


def resample_walls(walls, cells, mode=None):
    """Occupancy map ``walls`` cut into ``cells``, as ``RASTERIZE`` says."""
    from scipy import ndimage
    mode = mode or RASTERIZE
    walls = np.asarray(walls, dtype=bool)
    if mode == "bilinear":
        return ndimage.zoom(walls.astype(float),
                            (cells[0] / walls.shape[0],
                             cells[1] / walls.shape[1]), order=1) > 0.5
    if mode != "conservative":
        raise ValueError("unknown rasterisation %r" % (mode,))

    def spans(n_src, n_dst):
        # Source cells [j, j + 1) overlapping destination cell [i s, (i + 1) s).
        s = n_src / n_dst
        lo = np.floor(np.arange(n_dst) * s + 1e-9).astype(int)
        hi = np.ceil((np.arange(n_dst) + 1) * s - 1e-9).astype(int)
        return lo, np.maximum(hi, lo + 1)

    (lx, hx), (ly, hy) = spans(walls.shape[0], cells[0]), spans(walls.shape[1], cells[1])
    out = np.zeros(tuple(cells), dtype=bool)
    for i in range(cells[0]):
        rows = walls[lx[i]:hx[i]].any(axis=0)
        for j in range(cells[1]):
            out[i, j] = rows[ly[j]:hy[j]].any()
    return out


def resolution_scene(name, cells=None):
    """One scene of the resolution study, cut into ``cells``.

    Returns the environment, the start and the goal, all in the frame's units,
    so a path measured at one resolution is comparable with the next.
    """
    from scipy import ndimage
    source, zoom, frame = RESOLUTION_SCENES[name]
    if source.startswith("map:"):
        from PIL import Image
        img = np.array(Image.open(os.path.join(MAPS, source[4:] + ".png")
                                  ).convert("L")) < 128
        walls = ndimage.zoom(img.astype(float), (2, 2), order=1) > 0.5
        seed, tip = SCENARIOS["S1"]["seed"], SCENARIOS["S1"]["tip"]
    else:
        run = B.load_campaign(source)["runs"][0]
        walls = np.asarray(from_benchmark_run(run, zoom=zoom, n_theta=2,
                                              vehicle=DEFAULT_VEHICLE).walls)
        st = run["settings"]["env"]
        seed = [st["start"]["x"], st["start"]["y"], st["start"]["theta"], 0]
        tip = [st["goal"]["x"], st["goal"]["y"], st["goal"]["theta"], 0]
    if cells is None:
        cells = walls.shape
    if tuple(cells) != walls.shape:
        walls = resample_walls(walls, cells)
    dxy = [frame[0] / cells[0], frame[1] / cells[1]]
    env = from_occupancy(walls, zoom=zoom, n_theta=100, dxy=dxy,
                         vehicle=DEFAULT_VEHICLE)
    return env, seed, tip


def resolution_cells(name, nx):
    full, _, _ = resolution_scene(name)
    shape = np.asarray(full.walls).shape
    return nx, int(round(nx * shape[1] / shape[0]))


# --------------------------------------------------------------------- parts

def part_envs(out, resume=False):
    """The Two-Rooms scene file the navigation-function figure reads."""
    write_env(os.path.join(out, "env_two_rooms.npz"), hundred_square())


def part_resolution(out, resume=False):
    """The grid-resolution study: the same problem on six grids.

    ``gpu_mb`` is what the cupy memory pool holds once the solve is done,
    measured from an emptied pool, in units of 10^6 bytes.

    The curvature columns are measured on the geodesic as the solver returns
    it, goal to start, as the comparison plates do; the two deviations, which
    look at the final poses, on the path in the order it is driven.
    """
    import cupy
    from shfm.metrics import (summarize, quiet, angular_deviation_at_goal,
                              euclidean_deviation_at_goal)
    os.makedirs(out, exist_ok=True)
    pool = cupy.get_default_memory_pool()
    rows, geos = [], {}
    cols = ["scenario", "Nx", "Ny", "Ntheta", "npoints", "degenerate",
            "solve_s", "gpu_mb", "geo_pts", "seed_reach", "path_length", "aol",
            "max_curvature", "normalized_curvature", "min_clearing",
            "mean_clearing", "ang_dev", "euc_dev"]
    # The first solve of a process also sets up the CUDA context and the
    # kernel module, about a second on the T1200; we discard one solve to keep
    # that out of the first measured point.
    name = next(iter(RESOLUTION_SCENES))
    nx = RESOLUTION_STEPS[0]
    env, seed, tip = resolution_scene(name, cells=(nx, nx))
    sweep_solve(env, seed, tip, BASE, **RESOLUTION_SOLVE[name])
    for name in RESOLUTION_SCENES:
        zoom = RESOLUTION_SCENES[name][1]
        full, seed0, tip0 = resolution_scene(name)
        shape = np.asarray(full.walls).shape
        geos["%s__walls" % name] = np.asarray(full.walls)
        geos["%s__X0" % name] = np.asarray(full.X[0])
        geos["%s__X1" % name] = np.asarray(full.X[1])
        geos["%s__zoom" % name] = float(zoom)
        geos["%s__seed" % name] = np.asarray(seed0, dtype=float)
        geos["%s__tip" % name] = np.asarray(tip0, dtype=float)
        for nx in RESOLUTION_STEPS:
            ny = int(round(nx * shape[1] / shape[0]))
            env, seed, tip = resolution_scene(name, cells=(nx, ny))
            pool.free_all_blocks()
            base = pool.total_bytes()
            t0 = time.time()
            geo = sweep_solve(env, seed, tip, BASE, **RESOLUTION_SOLVE[name])
            dt = time.time() - t0
            gpu_mb = (pool.total_bytes() - base) / 1e6
            # A geodesic can come back with many points and still stop
            # short, when the backtracking runs into an obstacle on a coarse
            # grid; one that ends further than a vehicle length from either
            # end is not a path.
            reach = reach_error(geo, seed, tip)
            degenerate = int(geo.ndim != 2 or geo.shape[1] < 2
                             or not reach <= zoom * DEFAULT_VEHICLE.length)
            geos["%s__%dx%dx100" % (name, nx, ny)] = geo.T.astype(np.float32)
            row = dict(scenario=name, Nx=nx, Ny=ny, Ntheta=100,
                       npoints=nx * ny * 100, degenerate=degenerate,
                       solve_s=dt, gpu_mb=gpu_mb,
                       geo_pts=int(geo.shape[1]) if geo.ndim == 2 else 0,
                       seed_reach=reach)
            if not degenerate:
                raw = geo[:3].T
                driven = raw[::-1]
                with quiet():
                    st = summarize(raw, env, tip[:3])
                    ang_dev = angular_deviation_at_goal(driven, tip[2])
                    euc_dev = euclidean_deviation_at_goal(driven, tip[:2])
                row.update(path_length=st["length"], aol=st["aol"],
                           max_curvature=st["max_curvature"],
                           normalized_curvature=st["normalized_curvature"],
                           min_clearing=st["min_clearing"],
                           mean_clearing=st["mean_clearing"],
                           ang_dev=ang_dev, euc_dev=euc_dev)
            else:
                row.update({k: float("nan") for k in cols[10:]})
            rows.append(row)
            log("  %-15s %3dx%3dx100  %.2f s  length %s"
                % (name, nx, ny, dt,
                   ("%.3f" % row["path_length"]) if not degenerate else "degenerate"))
    geos["rlength"] = DEFAULT_VEHICLE.length
    geos["rwidth"] = DEFAULT_VEHICLE.width
    np.savez_compressed(os.path.join(out, "sweep_resolution_geos.npz"), **geos)
    with open(os.path.join(out, "sweep_resolution.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})
    log("  wrote sweep_resolution.csv and sweep_resolution_geos.npz")


def peak_memory(name, nx):
    """Peak GPU memory in use during one solve of the resolution study."""
    import cupy

    class Peak(cupy.cuda.MemoryHook):
        name = "peak"

        def __init__(self):
            self.cur = self.peak = 0
            self.sizes = {}

        def malloc_postprocess(self, device_id, size, mem_size, mem_ptr, pmem_id):
            self.sizes[mem_ptr] = mem_size
            self.cur += mem_size
            self.peak = max(self.peak, self.cur)

        def free_postprocess(self, device_id, mem_size, mem_ptr, pmem_id):
            self.cur -= self.sizes.pop(mem_ptr, mem_size)

    pool = cupy.get_default_memory_pool()
    env, seed, tip = resolution_scene(name, cells=resolution_cells(name, nx))
    pool.free_all_blocks()
    base = pool.total_bytes()
    h = Peak()
    with h:
        sweep_solve(env, seed, tip, BASE, **RESOLUTION_SOLVE[name])
    return dict(scene=name, n=nx, pool_mb=(pool.total_bytes() - base) / 1e6,
                peak_used_mb=h.peak / 1e6)


def part_memory(out, resume=False):
    """``peak_memory`` of every solve of ``resolution``, each in a new process."""
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "memory.jsonl")
    with open(path, "w") as fh:
        for name in RESOLUTION_SCENES:
            for nx in RESOLUTION_STEPS:
                done = subprocess.run(
                    [sys.executable, os.path.abspath(__file__), "--peak-memory",
                     name, str(nx), "--rasterize", RASTERIZE],
                    capture_output=True, text=True, check=True)
                line = done.stdout.strip().splitlines()[-1]
                fh.write(line + "\n")
                log("  %s" % line)


SWEPT_SCENES = {"env1": ("two_rooms", 8.0), "warehouse": ("warehouse", 2.0),
                "procedural": ("procedural", 2.0)}
"""The three scenes of the swept-area comparison, under the names it uses."""


def part_reference_swept(out, resume=False):
    """Floor area swept by the trajectories the sampling planners returned."""
    from shfm.metrics import swept_area, quiet
    bench = []
    for label, (scene, zoom) in SWEPT_SCENES.items():
        data = B.load_campaign(scene)
        for run_id, run in enumerate(data["runs"]):
            env = from_benchmark_run(run, zoom=zoom, n_theta=2,
                                     vehicle=DEFAULT_VEHICLE)
            for planner, plan in run["plans"].items():
                traj = plan.get("trajectory")
                if not traj:
                    continue
                with quiet():
                    area = swept_area(np.asarray(traj), env.vehicle, zoom)
                bench.append(dict(json=scene, env=label, zoom=zoom,
                                  run_id=run_id, planner=planner,
                                  swept_area_m2=area))
            log("  %-11s run %2d" % (label, run_id))
    path = os.path.join(out, "benchmr_swept.csv")
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["json", "env", "zoom", "run_id",
                                           "planner", "swept_area_m2"])
        w.writeheader()
        w.writerows(bench)
    log("  wrote %s (%d rows)" % (os.path.basename(path), len(bench)))


def _exact_sweep(geos, rows, tag, env, seed, tip, key_field):
    corners = np.asarray(env.vehicle.corners(env.zoom))
    mpu = env.vehicle.metres_per_unit(env.zoom)
    shape = env.walls.shape
    for param, values in EXACT_VALUES.items():
        for value in values:
            key = "%s__%s__%s" % (tag, param, float(value))
            if key in geos:
                continue
            params = dict(BASE, **{param: float(value)})
            t0 = time.time()
            try:
                geo = sweep_solve(env, seed, tip, params, saturate=True,
                                  free_start=True, extra=PASTSEED_200)
            except Exception as exc:                            # noqa: BLE001
                log("  %-40s FAILED %r" % (key, exc))
                continue
            if geo.ndim != 2 or geo.shape[1] < 2:
                log("  %-40s empty" % key)
                continue
            geos[key] = geo.astype(np.float32)
            sm = Q.state_metrics(geo.T[::-1])
            rows.append({key_field: tag, "param": param, "value": float(value),
                         "n_points": int(geo.shape[1]),
                         "swept_area_m2": swept_area_m2(geo, corners, mpu, shape),
                         "reach_error": reach_error(geo, seed, tip),
                         "path_length": float(np.sum(np.linalg.norm(
                             np.diff(geo[:2].T, axis=0), axis=1))),
                         "frac_backward_length": sm.get("frac_backward_length"),
                         "n_cusps": sm.get("n_cusps"),
                         "frac_man": sm.get("frac_man_fwd", 0) + sm.get("frac_man_bwd", 0),
                         "solve_s": round(time.time() - t0, 2)})
            log("  %-40s %.1f s" % (key, time.time() - t0))


def _save_sweep(directory, stem, geos, rows, key_field):
    cols = [key_field, "param", "value", "n_points", "swept_area_m2", "reach_error",
            "path_length", "frac_backward_length", "n_cusps", "frac_man", "solve_s"]
    np.savez_compressed(os.path.join(directory, "geos_1d_%s.npz" % stem), **geos)
    with open(os.path.join(directory, "swept_1d_%s.csv" % stem), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})


def part_one_parameter(out, resume=False):
    """One parameter moved at a time around ``BASE``, on ``SITUATIONS`` at
    200 cells a side; clearance capped at the vehicle length, start mode free.

    Writes ``env_two_rooms.npz`` (the scene), ``geos_1d_two_rooms.npz`` (the
    geodesics as the solver returns them, ``(4, n)``, goal to start) and
    ``swept_1d_two_rooms.csv`` (one row per solve).
    """
    os.makedirs(out, exist_ok=True)
    env = two_hundred_square()
    write_env(os.path.join(out, "env_two_rooms.npz"), env)
    geo_path = os.path.join(out, "geos_1d_two_rooms.npz")
    csv_path = os.path.join(out, "swept_1d_two_rooms.csv")
    geos, rows = {}, []
    if resume and os.path.exists(geo_path):
        geos = dict(np.load(geo_path).items())
        rows = list(csv.DictReader(open(csv_path)))
    for tag, seed, tip in situations_200():
        log("situation %s" % tag)
        _exact_sweep(geos, rows, tag, env, seed, tip, "scenario")
        _save_sweep(out, "two_rooms", geos, rows, "scenario")


def part_k1_k2_map(out, resume=False):
    """The (k1, k2) grid ``MAP_GRID``: ``SITUATIONS`` at 200 cells a side, then
    the docking and Narrow-Passage problems of the campaigns. 4WS, reference
    speed coefficients, clearance capped at the vehicle length, start mode
    free."""
    store = Q.Store(out, resume)
    env = two_hundred_square()
    for tag, seed, tip in situations_200():
        for k1, k2 in MAP_GRID:
            T = transitions_of(k1, k2)
            key = "%s__k1_%g__k2_%g" % (tag, k1, k2)
            Q.attempt(store, key,
                      lambda seed=seed, tip=tip, T=T:
                      Q.solve(env, seed, tip, "4WS", 0.5, transitions=T,
                              coefficients=MAP_COEFFICIENTS, saturate=True, robust=True,
                              free_modes="start", **PASTSEED_200)
                      + (env, seed, tip),
                      dict(scene=tag, k1=k1, k2=k2))
        store.flush()
    for name in ("docking", "narrow_passage"):
        env, seed, tip = Q.campaign_problem(name)
        delta = Q.CAMPAIGN_SCENES[name][1]
        for k1, k2 in MAP_GRID:
            T = transitions_of(k1, k2)
            key = "%s__k1_%g__k2_%g" % (name, k1, k2)
            Q.attempt(store, key,
                      lambda env=env, seed=seed, tip=tip, delta=delta, T=T:
                      Q.solve(env, seed, tip, "4WS", delta, transitions=T,
                              coefficients=MAP_COEFFICIENTS, saturate=True, robust=True,
                              free_modes="start", **Q.PASTSEED)
                      + (env, seed, tip),
                      dict(scene=name, k1=k1, k2=k2))
        store.flush()


def part_recommended(out, resume=False):
    """(2, 3) and (10, 50), solved as ``one_parameter`` solves, on ``SITUATIONS``."""
    env = two_hundred_square()
    corners = np.asarray(env.vehicle.corners(env.zoom))
    mpu = env.vehicle.metres_per_unit(env.zoom)
    store = Q.Store(out, resume)
    for tag, seed, tip in situations_200():
        for k1, k2 in ((10.0, 50.0), (2.0, 3.0)):
            key = "%s__k1_%g__k2_%g" % (tag, k1, k2)
            if store.has(key):
                continue
            geo = sweep_solve(env, seed, tip, dict(BASE, k1=k1, k2=k2), saturate=True,
                              free_start=True, extra=PASTSEED_200)
            sm = Q.state_metrics(geo.T[::-1])
            store.add(key, dict(scene=tag, k1=k1, k2=k2,
                                swept_area_m2=swept_area_m2(geo, corners, mpu, env.walls.shape),
                                reach_error=reach_error(geo, seed, tip),
                                n_cusps=sm.get("n_cusps"),
                                frac_man=sm.get("frac_man_fwd", 0) + sm.get("frac_man_bwd", 0),
                                frac_backward_length=sm.get("frac_backward_length")),
                      geo.T[::-1])
            log("  %s" % key)
    store.flush()


def part_clearance_cap(out, resume=False):
    """The cap on the obstacle cost, from no obstacle cost to no cap.

    The comparisons cap the clearance term at the vehicle length. Lowering
    the cap shortens the reach of the cost; at zero the path is a pure
    geodesic. The lift is computed once per map and only the cap is changed
    between solves. Transition costs (2, 3), start mode free.
    """
    store = Q.Store(out, resume)
    transitions = transition_matrix(2.0, 3.0)
    for scene in ("two_rooms", "warehouse", "procedural"):
        _, delta = Q.CAMPAIGN_SCENES[scene]
        for i in Q.distinct_runs(scene):
            env, seed, tip = Q.campaign_problem(scene, i)
            full = np.asarray(env.walls3_dist)
            for label, cap in SATURATION_CAPS:
                if cap is None:
                    kw = dict(saturate=False)
                elif cap == 0.0:
                    kw = dict(obstacle_cost=False)
                else:
                    env.walls3_dist_saturated = np.minimum(full, cap * env.zoom)
                    kw = {}
                for method in ("2WS", "4WS"):
                    key = "%s__run%02d__%s__%s" % (scene, i, label, method)
                    Q.attempt(store, key,
                              lambda kw=kw, method=method: Q.solve(
                                  env, seed, tip, method, delta, transitions=transitions,
                                  free_modes="start", **dict(kw, **Q.PASTSEED))
                              + (env, seed, tip),
                              dict(scene=scene, run=i, cap=label,
                                   cap_m="" if cap is None else cap, method=method))
            store.flush()
            del env, full


PARTS = {"envs": part_envs, "resolution": part_resolution, "memory": part_memory,
         "reference_swept": part_reference_swept, "one_parameter": part_one_parameter,
         "k1_k2_map": part_k1_k2_map, "recommended": part_recommended,
         "clearance_cap": part_clearance_cap}
TARGET = {"envs": "", "reference_swept": "", "memory": "resolution"}
"""Subdirectory of the output each part writes into, when not its own name."""


def main():
    global OUT, RASTERIZE
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", nargs="+", default=list(PARTS), choices=list(PARTS))
    ap.add_argument("--resume", action="store_true",
                    help="keep the solves a previous run already wrote")
    ap.add_argument("--rasterize", default=RASTERIZE,
                    choices=["conservative", "bilinear"],
                    help="how the resolution study resamples the map")
    ap.add_argument("--out", default=OUT,
                    help="output directory (default: runs/sensitivity; "
                         "data/sensitivity replaces the shipped files)")
    ap.add_argument("--peak-memory", nargs=2, metavar=("SCENE", "N"),
                    help=argparse.SUPPRESS)
    args = ap.parse_args()
    RASTERIZE = args.rasterize
    if args.peak_memory:
        print(json.dumps(peak_memory(args.peak_memory[0], int(args.peak_memory[1]))))
        return
    OUT = os.path.abspath(args.out)
    os.makedirs(OUT, exist_ok=True)
    for name in args.part:
        target = os.path.join(OUT, TARGET.get(name, name))
        Q.log("=== part %s" % name)
        t0 = time.time()
        PARTS[name](target, args.resume)
        Q.log("=== part %s done in %.0f s" % (name, time.time() - t0))


if __name__ == "__main__":
    main()
