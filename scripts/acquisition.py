"""Shared pieces of the acquisition scripts.

A study is split into parts. Each part writes, in its own directory,
``rows.csv`` (one row per solve, metrics included) and ``paths.npz`` (the
paths, in the order they are driven, ``(n, 4)``: x, y, theta, mode index in
solver order). A part can be interrupted and resumed.

Needs a CUDA device and the Bench-MR metric bindings (``scripts/setup.sh``).
"""
import csv
import os
import sys
import time
import traceback

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from shfm import DEFAULT_VEHICLE, Planner, REFERENCE_COEFFICIENTS   # noqa: E402
from shfm import benchmark as B, metrics as M                       # noqa: E402
from shfm.environments import from_benchmark_run                    # noqa: E402
from shfm.kinematics import control_set, control_set_2ws            # noqa: E402
from shfm.states import SOLVER_ORDER                                # noqa: E402

ROBUST = dict(nitermax_o=20000, multiprecision=True,
              raiseOnNonConvergence=False)
"""Solver flags of the parameter sweeps, which visit degenerate settings."""

CAMPAIGN_SCENES = {
    # name             zoom  steering saturation (rad)
    "two_rooms":      (8.0, 0.5),
    "warehouse":      (2.0, 0.5),
    "procedural":     (2.0, 0.5),
    "docking":        (20.0, 1.05),
    "narrow_passage": (8.0, 0.5),
}
"""Scale and steering saturation each campaign scene is solved with."""

PASTSEED = dict(geodesic_PastSeed_delay=128)
"""Delay of agd's PastSeed criterion, in steps (8 by default). The criterion
stops the backtracking when the distance to the seed has grown over that many
steps; a manoeuvre close to the start makes it grow for a while, and the path
then stops a few cells short of the start. 128 is the value of the comparison
studies; the parameter study uses 1024 on its Two-Rooms situations
(``acquire_sensitivity.PASTSEED_200``). ``close_start`` joins a path that
still stops short."""


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ---------------------------------------------------------------- storage

class Store:
    """``rows.csv`` and ``paths.npz`` of one part, written as it goes."""

    def __init__(self, directory, resume):
        self.dir = directory
        os.makedirs(self.dir, exist_ok=True)
        self.csv = os.path.join(self.dir, "rows.csv")
        self.npz = os.path.join(self.dir, "paths.npz")
        self.rows, self.paths = [], {}
        if resume and os.path.exists(self.csv):
            self.rows = list(csv.DictReader(open(self.csv)))
            if os.path.exists(self.npz):
                self.paths = dict(np.load(self.npz).items())
            log("  %s: resuming, %d rows" % (os.path.basename(directory), len(self.rows)))
        self.done = {r["key"] for r in self.rows}
        self._dirty = 0

    def has(self, key):
        return key in self.done

    def add(self, key, row, path=None):
        row = dict(key=key, **row)
        self.rows.append(row)
        self.done.add(key)
        if path is not None:
            self.paths[key] = np.asarray(path, dtype=np.float32)
        self._dirty += 1
        if self._dirty >= 10:
            self.flush()

    def flush(self):
        cols = []
        for r in self.rows:
            for k in r:
                if k not in cols:
                    cols.append(k)
        tmp = self.csv + ".tmp"
        with open(tmp, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            for r in self.rows:
                w.writerow({c: r.get(c, "") for c in cols})
        os.replace(tmp, self.csv)
        tmp = self.npz + ".tmp.npz"
        np.savez_compressed(tmp, **self.paths)
        os.replace(tmp, self.npz)
        self._dirty = 0


# ---------------------------------------------------------------- solving

LAST = {}
"""What the last :func:`solve` reported besides the path: the solver's
stopping criterion for the backtracking and, with a free goal mode, the mode
chosen at the goal. :func:`attempt` copies it into the row."""


def solve(env, seed, tip, method="4WS", delta=0.5, transitions=None,
          coefficients=None, saturate=True, obstacle_cost=True, robust=False,
          free_modes=None, **solver_kw):
    """One solve of a campaign problem.

    ``free_modes`` is None (start and goal in the mode they name), ``"start"``
    or ``"both"``: see ``free_start_mode`` and ``free_goal_mode`` in
    :meth:`shfm.Planner.solve`.

    Returns ``(driven, raw, seconds)``: the path as driven (start to goal) and
    as the solver produced it (goal to start), both ``(n, 4)``.
    """
    import cupy
    coefficients = dict(REFERENCE_COEFFICIENTS, **(coefficients or {}))
    builder = control_set if method == "4WS" else control_set_2ws
    mk = lambda th, wb, xp: builder(th, wb, xp=xp, delta_f_max=delta,
                                    delta_r_max=delta, **coefficients)
    planner = Planner(env, controls=mk, transitions=transitions,
                      saturate=saturate, obstacle_cost=obstacle_cost)
    kw = dict(ROBUST) if robust else {}
    kw.update(solver_kw)
    if free_modes in ("start", "both"):
        kw["free_start_mode"] = True
    if free_modes == "both":
        kw["free_goal_mode"] = True
    t0 = time.time()
    r = planner.solve(list(seed), [list(tip)], **kw)
    dt = time.time() - t0
    driven, raw = r.path(0), r.raw_path(0)
    LAST.clear()
    stops = r.hfm_out.get("geodesic_stopping_criteria")
    if stops:
        LAST["stop"] = str(stops[0])
    if "goal_modes" in r.hfm_out:
        LAST["goal_mode"] = int(r.hfm_out["goal_modes"][0])
    del r
    cupy.get_default_memory_pool().free_all_blocks()
    return driven, raw, dt


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def state_metrics(path):
    """Mode use, switches and cusps along a driven path ``(n, 4)``.

    A cusp is a change of the direction of travel relative to the heading:
    the sign of the scalar product between the displacement and the heading
    vector flips. In manoeuvring the displacement is turned by the rear
    steering angle, below a right angle, so the sign still reads the direction.
    """
    p = np.asarray(path, dtype=float)
    out = {}
    if p.ndim != 2 or len(p) < 2:
        return out
    modes = np.rint(p[:, 3]).astype(int)
    for k, name in enumerate(SOLVER_ORDER):
        out["frac_" + name] = float(np.mean(modes == k))
    out["n_mode_switches"] = int(np.sum(modes[1:] != modes[:-1]))
    d = np.diff(p[:, :2], axis=0)
    step = np.linalg.norm(d, axis=1)
    h = np.c_[np.cos(p[:-1, 2]), np.sin(p[:-1, 2])]
    dot = np.sum(d * h, axis=1)
    s = np.sign(dot[step > 1e-6])
    out["n_cusps"] = int(np.sum(s[1:] != s[:-1])) if len(s) > 1 else 0
    seg = step.sum()
    out["frac_backward_length"] = float(step[dot < 0].sum() / seg) if seg > 0 else float("nan")
    return out


def path_metrics(driven, raw, env, seed, tip):
    """Bench-MR metrics, as the plates compute them, plus the state metrics.

    Curvature on the raw output (as :func:`shfm.plates.custom_stats`), the
    rest on the driven path. ``seed_gap`` and ``tip_gap`` are the distances,
    in grid units, between the ends of the path and the configurations asked
    for; ``seed_dtheta`` and ``tip_dtheta`` the heading gaps, in radians.
    """
    p = np.asarray(driven, dtype=float)
    out = dict(n_points=len(p))
    if len(p) < 2:
        out["degenerate"] = 1
        return out
    with M.quiet():
        mx, norm = M.curvature(np.asarray(raw, dtype=float)[:, :3])
        clear = M.clearing_distances(p[:, :3], env.walls)
        out.update(
            path_length=M.path_length(p[:, :3]),
            max_curvature=mx, normalized_curvature=norm,
            aol=M.aol(p[:, :3]),
            min_clearing=float(clear.min()), mean_clearing=float(clear.mean()),
            swept_area_m2=M.swept_area(p[:, :3], env.vehicle, env.zoom),
            angular_deviation=M.angular_deviation_at_goal(p[:, :3], tip[2]),
            euclidean_deviation=M.euclidean_deviation_at_goal(p[:, :3], tip[:2]))
    out["seed_gap"] = float(np.hypot(*(p[0, :2] - np.asarray(seed[:2]))))
    out["seed_dtheta"] = float(abs(wrap(p[0, 2] - seed[2])))
    out["tip_gap"] = float(np.hypot(*(p[-1, :2] - np.asarray(tip[:2]))))
    out["tip_dtheta"] = float(abs(wrap(p[-1, 2] - tip[2])))
    reach = env.vehicle.length * env.zoom
    out["degenerate"] = int(not (out["seed_gap"] <= reach and out["tip_gap"] <= reach))
    out.update(state_metrics(p))
    return out


def campaign_problem(scene, run_index=0, saturation_cap=None):
    """Environment, start and goal of one run of a campaign."""
    zoom, _ = CAMPAIGN_SCENES[scene]
    run = B.load_campaign(scene)["runs"][run_index]
    env = from_benchmark_run(run, zoom=zoom, n_theta=100,
                             vehicle=DEFAULT_VEHICLE,
                             saturation_cap=saturation_cap)
    start, goal = B.endpoints_of(run)
    return env, list(start) + [0], list(goal) + [0]


def distinct_runs(scene):
    """Run indices with a distinct (map, start, goal); one for fixed scenes."""
    data = B.load_campaign(scene)
    seen, keep = set(), []
    for i, run in enumerate(data["runs"]):
        s, g = B.endpoints_of(run)
        key = (run["environment"]["map"], tuple(s), tuple(g))
        if key not in seen:
            seen.add(key)
            keep.append(i)
    return keep


def attempt(store, key, fn, meta):
    """Run ``fn() -> (driven, raw, dt, env, seed, tip)`` once and record it."""
    if store.has(key):
        return
    try:
        LAST.clear()
        driven, raw, dt, env, seed, tip = fn()
        row = dict(meta, solve_s=round(dt, 3), **LAST)
        row.update(path_metrics(driven, raw, env, seed, tip))
        store.add(key, row, driven)
        log("  %-50s %6.2f s  L=%s" % (key, dt, ("%.1f" % row["path_length"])
                                        if "path_length" in row else "-"))
    except Exception as exc:                                    # noqa: BLE001
        store.add(key, dict(meta, error=repr(exc)[:200]))
        log("  %-50s FAILED %r" % (key, exc))
        traceback.print_exc()


def run_parts(parts, chosen, out, resume):
    """Run the parts named in ``chosen``, each into ``out/<part>``."""
    for name in chosen:
        log("=== part %s" % name)
        t0 = time.time()
        try:
            parts[name](os.path.join(out, name), resume)
        except Exception:                                       # noqa: BLE001
            log("part %s stopped on an error" % name)
            traceback.print_exc()
        log("=== part %s done in %.0f s" % (name, time.time() - t0))
