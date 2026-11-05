"""Re-run the benchmark campaigns and write them to ``runs/``.

The campaigns in ``data/campaigns/`` are the ones every figure reads. This
produces fresh ones beside them, with the same settings, to check that the
pipeline still works end to end rather than to redraw a given figure.

    python scripts/regenerate_campaigns.py                 # every scene
    python scripts/regenerate_campaigns.py two_rooms       # one of them

RRT stops at its first solution and is reproduced to the last digit, except
for its normalised curvature (``external/NOTICE.md``). The four anytime
planners explore until their time budget expires, so they return the same
distribution and different paths. ``scripts/verify_campaigns.py`` measures
that, run by run.

Needs the benchmark binary; build it with ``scripts/setup.sh``.
"""
import glob
import gzip
import json
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from shfm import benchmark as B, campaigns as C   # noqa: E402

def slim(data):
    """Drop what no figure reads, which is most of the file by volume."""
    out = {"globals": data.get("globals"), "settings": data.get("settings"),
           "runs": []}
    for i, run in enumerate(data["runs"]):
        kept = {"environment": run["environment"],
                "settings": run.get("settings"), "plans": {}}
        for name, plan in run["plans"].items():
            if plan is None:
                kept["plans"][name] = None
                continue
            entry = {"stats": plan.get("stats"), "path": plan.get("path"),
                     "smoothing": None}
            if plan.get("trajectory"):
                entry["trajectory"] = plan["trajectory"]
            kept["plans"][name] = entry
        out["runs"].append(kept)
    return out


def merge(workdir, scene):
    """Merge the per-planner result files the pool wrote."""
    mpb = C._mpb_module()
    from mpb import MPB
    files = sorted(glob.glob(os.path.join(workdir, scene,
                                          "%s_*_results.json" % scene)))
    if not files:
        raise RuntimeError("no result file under %s" % workdir)
    jobs = []
    for path in files:
        job = MPB()
        job.results_filename = path
        jobs.append(job)
    merged = os.path.join(workdir, "%s_merged.json" % scene)
    MPB.merge(jobs, merged, silence=True)
    return merged


def regenerate(scene):
    runs = C.n_runs(scene)
    workdir = tempfile.mkdtemp(prefix="shfm-regen-%s-" % scene)
    print("%s : %d runs, %d planners" % (scene, runs, len(C.PLANNERS)))
    t0 = time.time()
    C.regenerate(scene, runs=runs, workdir=workdir)
    merged = merge(workdir, scene)
    data = C.strip_paths(slim(json.load(open(merged))))
    out_dir = os.path.join(HERE, "runs")
    os.makedirs(out_dir, exist_ok=True)
    target = os.path.join(out_dir, scene + ".json.gz")
    with gzip.open(target, "wt", encoding="utf-8", compresslevel=9) as f:
        json.dump(data, f)
    print("  -> %s  (%d runs, %.2f MB, %.0f s)"
          % (target, len(data["runs"]), os.path.getsize(target) / 1e6,
             time.time() - t0))
    shutil.rmtree(workdir, ignore_errors=True)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", nargs="*", default=None, choices=list(B.CAMPAIGNS),
                    help="campaigns to re-run (default: all five)")
    for scene in (ap.parse_args().scenes or list(B.CAMPAIGNS)):
        regenerate(scene)
