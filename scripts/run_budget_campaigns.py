#!/usr/bin/env python3
"""Re-run the benchmark campaigns with another planning time budget.

The campaigns of ``data/campaigns/`` give the sampling planners 15 s (5 s on
Warehouse). A shorter budget, 1 s, leaves the anytime planners less time to
improve their paths. This re-runs the campaigns, every setting unchanged
except ``max_planning_time``, so that the two can be compared on fresh
executions:

    python scripts/run_budget_campaigns.py --budget 1
    python scripts/run_budget_campaigns.py --budget original   # control run

Each scene is written to ``runs/comparison/budget/<scene>_<budget>.json.gz``
with the trajectory of every run, so that every metric, including the swept
area, can be recomputed (``generators/budget_table.py``). Run it on an
otherwise idle machine: the planners are given a wall-clock budget.

The five planners of a scene run side by side, one process each, as
``MultipleMPB.run_parallel`` ran them; the binary is called directly so that
no watchdog kills a slow run (``mpb`` kills a planner after twice the budget
times the number of runs, which a 1 s budget with smoothing can exceed).
"""
import argparse
import concurrent.futures as cf
import copy
import gzip
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from shfm import benchmark as B, campaigns as C    # noqa: E402

OUT = os.path.join(HERE, "runs", "comparison", "budget")
SCENES = ("two_rooms", "warehouse", "procedural", "docking")


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def config_for(scene, planner, budget):
    """The campaign configuration for one planner, budget substituted."""
    job = C.make_mpb(scene, planner)
    cfg = copy.deepcopy(job.config)
    if budget is not None:
        cfg["max_planning_time"] = float(budget)
    cfg["benchmark"]["runs"] = C.n_runs(scene)
    for key in cfg["benchmark"]["planning"]:
        cfg["benchmark"]["planning"][key] = (key == planner)
    return cfg


def run_planner(scene, planner, budget, workdir):
    """One planner over every run of a scene, in a directory of its own."""
    os.makedirs(workdir, exist_ok=True)
    for name in ("polygon_mazes", "sbpl_mprim", "image_mazes"):
        src = os.path.join(B.BENCH_ROOT, "bin", name)
        dst = os.path.join(workdir, name)
        if os.path.isdir(src) and not os.path.exists(dst):
            os.symlink(src, dst)
    cfg = config_for(scene, planner, budget)
    cfg["benchmark"]["log_file"] = os.path.join(workdir, "results.json")
    with open(os.path.join(workdir, "config.json"), "w") as f:
        json.dump({"settings": cfg}, f, indent=1)
    t0 = time.time()
    with open(os.path.join(workdir, "stdout.log"), "w") as out:
        proc = subprocess.run([B.BINARY, "config.json"], cwd=workdir,
                              stdout=out, stderr=subprocess.STDOUT)
    res = os.path.join(workdir, "results.json")
    if not os.path.exists(res):
        raise RuntimeError("%s/%s: no results (exit %s), see %s"
                           % (scene, planner, proc.returncode, workdir))
    with open(res) as f:
        data = json.load(f)
    return data, time.time() - t0


def merge(parts):
    """One result file from the per-planner ones, run by run."""
    first = next(iter(parts.values()))
    out = {"settings": first.get("settings"), "runs": []}
    n = max(len(d["runs"]) for d in parts.values())
    for i in range(n):
        run = {"environment": None, "settings": None, "plans": {}}
        for planner, data in parts.items():
            if i >= len(data["runs"]):
                continue
            r = data["runs"][i]
            run["environment"] = run["environment"] or r.get("environment")
            run["settings"] = run["settings"] or r.get("settings")
            for name, plan in r["plans"].items():
                if plan is None:
                    run["plans"][name] = None
                    continue
                run["plans"][name] = {"stats": plan.get("stats"),
                                      "path": plan.get("path"),
                                      "trajectory": plan.get("trajectory")}
        out["runs"].append(run)
    return out


def run_scene(scene, budget):
    tag = "original" if budget is None else "%gs" % budget
    target = os.path.join(OUT, "%s_%s.json.gz" % (scene, tag))
    if os.path.exists(target):
        log("%s %s: already there" % (scene, tag))
        return
    eff = C.settings(scene)["max_planning_time"] if budget is None else budget
    log("%s: %d runs x %d planners, budget %g s"
        % (scene, C.n_runs(scene), len(C.PLANNERS), eff))
    root = tempfile.mkdtemp(prefix="shfm-budget-%s-%s-" % (scene, tag))
    parts, t0 = {}, time.time()
    with cf.ThreadPoolExecutor(len(C.PLANNERS)) as pool:
        futures = {pool.submit(run_planner, scene, p, budget,
                               os.path.join(root, p)): p for p in C.PLANNERS}
        for fut in cf.as_completed(futures):
            p = futures[fut]
            try:
                parts[p], dt = fut.result()
                log("  %-18s %3.0f s" % (p, dt))
            except Exception as exc:                            # noqa: BLE001
                log("  %-18s FAILED %s" % (p, exc))
    if not parts:
        log("%s %s: nothing to merge" % (scene, tag))
        return
    data = C.strip_paths(merge(parts))
    data["budget_s"] = eff
    os.makedirs(OUT, exist_ok=True)
    with gzip.open(target + ".tmp", "wt", encoding="utf-8", compresslevel=6) as f:
        json.dump(data, f)
    os.replace(target + ".tmp", target)
    log("  -> %s (%.1f MB, %.0f s)" % (os.path.relpath(target, HERE),
                                       os.path.getsize(target) / 1e6,
                                       time.time() - t0))
    shutil.rmtree(root, ignore_errors=True)


def main():
    global OUT
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--budget", nargs="+", default=["1", "original"],
                    help="seconds, or 'original' for the campaign's own budget")
    ap.add_argument("--scenes", nargs="+", default=list(SCENES),
                    choices=list(B.CAMPAIGNS))
    ap.add_argument("--out", default=OUT,
                    help="output directory (default: runs/comparison/budget; "
                         "data/comparison/budget replaces the shipped files)")
    args = ap.parse_args()
    OUT = os.path.abspath(args.out)
    for b in args.budget:
        budget = None if b == "original" else float(b)
        for scene in args.scenes:
            run_scene(scene, budget)


if __name__ == "__main__":
    main()
