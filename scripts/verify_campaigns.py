"""Check that a campaign built here reproduces the shipped one, run by run.

The files in ``data/campaigns/`` are the reference. This rebuilds the same
campaign, one planner at a time, through Bench-MR's own ``MPB`` API and with
the settings of ``data/campaigns/settings.json`` (see
``shfm.campaigns.make_mpb``), then compares, run by run, the statistics of ``KEYS``: length, maximum
curvature, AOL, clearing distances and the two deviations at the goal.
RRT stops at its first solution and reproduces every statistic but the
normalised curvature, which the shipped campaigns computed with an earlier
build of Bench-MR (``external/NOTICE.md``); the anytime planners explore for
as long as their time budget allows, so they only reproduce when
the machine is loaded as it was.

It does not read ``runs/``: ``scripts/regenerate_campaigns.py`` is the one that
writes a full campaign there.

    python scripts/verify_campaigns.py two_rooms
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from shfm import benchmark as B, campaigns as C   # noqa: E402

# The normalised curvature is not compared: the shipped campaigns computed it
# with an earlier build of Bench-MR (external/NOTICE.md).
KEYS = ("path_length", "max_curvature", "aol",
        "min_clearing_distance", "mean_clearing_distance",
        "angular_deviation_metrics", "euclidean_deviation_metrics")

PLANNER_KEY = {"RRT": "rrt", "RRTstar": "rrt_star",
               "InformedRRTstar": "informed_rrt_star", "SST": "sst",
               "SPARS": "spars"}


def rebuild(scene, planner_key, runs, workdir):
    """Run one planner as its campaign was configured, through MPB.

    The configuration is the top block of the campaign, kept in
    ``settings.json``, with the map and the footprint resolved against the
    copies under ``data/``. The block stored inside each run of a result file
    is the state after that run, with the environment seed already advanced,
    and it names files by their absolute path on the machine that produced it.
    """
    br = os.path.join(HERE, "external", "bench-mr")
    for name in ("polygon_mazes", "sbpl_mprim", "image_mazes"):
        src = os.path.join(br, "bin", name)
        dst = os.path.join(workdir, name)
        if os.path.isdir(src) and not os.path.exists(dst):
            os.symlink(src, dst)
    cwd = os.getcwd()
    os.chdir(workdir)
    try:
        m = C.make_mpb(scene, planner_key)
        m.run(id="verify", runs=runs, show_progress_bar=False, silence=True,
              shuffle_planners=False)
        return json.load(open(m.results_filename))
    finally:
        os.chdir(cwd)


def main(name):
    ref = B.load_campaign(name)
    planners = sorted({k for r in ref["runs"] for k, v in r["plans"].items() if v})
    print("%s : %d runs, %s" % (name, len(ref["runs"]), ", ".join(planners)))
    workdir = tempfile.mkdtemp(prefix="verify-")
    ok = True
    for planner in planners:
        key = PLANNER_KEY.get(planner)
        if key is None:
            print("  %-17s unknown key" % planner)
            continue
        got = rebuild(name, key, len(ref["runs"]), workdir)
        same = 0
        for i in range(len(ref["runs"])):
            a = (got["runs"][i]["plans"].get(planner) or {}).get("stats") or {}
            b = (ref["runs"][i]["plans"].get(planner) or {}).get("stats") or {}
            if a and b and all(b.get(k) is None or abs(a.get(k, 0) - b[k]) < 1e-9
                               for k in KEYS):
                same += 1
        print("  %-17s %d/%d runs identical" % (planner, same, len(ref["runs"])))
        ok = ok and same == len(ref["runs"])
    print("=> %s" % ("IDENTICAL" if ok
                     else "differences on the anytime planners"))
    return 0 if ok else 1


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene", nargs="?", default="two_rooms", choices=list(B.CAMPAIGNS),
                    help="which campaign to rebuild and compare (default: two_rooms)")
    sys.exit(main(ap.parse_args().scene))
