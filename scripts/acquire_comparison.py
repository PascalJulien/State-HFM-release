#!/usr/bin/env python3
"""Acquisitions of our planner for the comparison with the sampling planners,
shipped under ``data/comparison/``.

    python scripts/acquire_comparison.py --part campaigns        # about 5 minutes
    python scripts/acquire_comparison.py --part modes            # about a minute
    python scripts/acquire_comparison.py --part reference_costs  # under a minute

Parts (each writes ``rows.csv`` and ``paths.npz`` into ``<out>/<part>/``):

* ``campaigns``: every problem of the five campaigns of ``data/campaigns/``
  (one per fixed scene, one per distinct procedural map), both vehicles, at
  the recommended transition costs k1 = 2, k2 = 3 and the reference speed
  coefficients.
* ``modes``: the Two-Rooms problem, both vehicles, for five steering
  saturations, 0.3 to 0.7 rad, given to both axles (the rear one only acts in
  the manoeuvring modes of the 4WS vehicle), and four ratios nu_2WS / nu_FWD.
* ``reference_costs``: the docking and Narrow-Passage problems at the centre of
  the parameter study, k1 = 10, k2 = 50.

Every solve seeds the start in every mode (``free_start_mode``) and delays the
PastSeed criterion of the backtracking (``acquisition.PASTSEED``). The paths
are stored as extracted; the figures remove the extraction chatter
(:func:`shfm.postprocess.dechatter`) and close the start on the requested pose
(:func:`shfm.splice.close_start`) before measuring them.

The output goes to ``runs/comparison/`` unless ``--out`` says otherwise.
Needs a CUDA device and the Bench-MR metric bindings (``scripts/setup.sh``).
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "scripts"))

from shfm.states import transition_matrix                           # noqa: E402

import acquisition as Q                                             # noqa: E402

OUT = os.path.join(HERE, "runs", "comparison")

RECOMMENDED = transition_matrix(2.0, 3.0)
"""Transition costs of the comparisons: k1 = 2, k2 = 3."""

SCENES = ("two_rooms", "docking", "warehouse", "narrow_passage", "procedural")


def part_campaigns(out, resume=False):
    """Both vehicles on every campaign problem, at (2, 3)."""
    store = Q.Store(out, resume)
    for scene in SCENES:
        _, delta = Q.CAMPAIGN_SCENES[scene]
        runs = Q.distinct_runs(scene) if scene == "procedural" else [0]
        for i in runs:
            env, seed, tip = Q.campaign_problem(scene, i)
            for method in ("4WS", "2WS"):
                key = "%s__run%02d__%s" % (scene, i, method)
                Q.attempt(store, key,
                          lambda method=method: Q.solve(
                              env, seed, tip, method, delta, transitions=RECOMMENDED,
                              free_modes="start", **Q.PASTSEED) + (env, seed, tip),
                          dict(scene=scene, run=i, k1=2.0, k2=3.0, method=method))
            store.flush()
            del env


def part_modes(out, resume=False):
    """Two-Rooms, both vehicles: the steering saturation, given to both axles
    (the rear one only acts in the 4WS manoeuvring modes), and the ratio
    nu_2WS / nu_FWD, at (2, 3)."""
    store = Q.Store(out, resume)
    env, seed, tip = Q.campaign_problem("two_rooms")
    for method in ("2WS", "4WS"):
        for d in (0.3, 0.4, 0.5, 0.6, 0.7):
            key = "two_rooms__%s__delta_%g" % (method, d)
            Q.attempt(store, key,
                      lambda d=d, method=method: Q.solve(
                          env, seed, tip, method, d, transitions=RECOMMENDED,
                          free_modes="start", **Q.PASTSEED) + (env, seed, tip),
                      dict(scene="two_rooms", method=method, sweep="delta", value=d))
        for ratio in (0.3, 0.5, 0.7, 0.9):
            co = dict(nu_2ws=3.0 * ratio)
            key = "two_rooms__%s__ratio2ws_%g" % (method, ratio)
            Q.attempt(store, key,
                      lambda co=co, method=method: Q.solve(
                          env, seed, tip, method, 0.5, coefficients=co,
                          transitions=RECOMMENDED, free_modes="start", **Q.PASTSEED)
                      + (env, seed, tip),
                      dict(scene="two_rooms", method=method, sweep="ratio_2ws_fwd", value=ratio))
        store.flush()


def part_reference_costs(out, resume=False):
    """Docking and Narrow-Passage at k1 = 10, k2 = 50, both vehicles."""
    store = Q.Store(out, resume)
    T = transition_matrix(10.0, 50.0)
    for scene in ("docking", "narrow_passage"):
        _, delta = Q.CAMPAIGN_SCENES[scene]
        env, seed, tip = Q.campaign_problem(scene, 0)
        for method in ("2WS", "4WS"):
            key = "%s__run00__%s" % (scene, method)
            Q.attempt(store, key,
                      lambda method=method: Q.solve(
                          env, seed, tip, method, delta, transitions=T,
                          free_modes="start", **Q.PASTSEED) + (env, seed, tip),
                      dict(scene=scene, run=0, k1=10.0, k2=50.0, method=method))
        store.flush()
        del env


PARTS = {"campaigns": part_campaigns, "modes": part_modes,
         "reference_costs": part_reference_costs}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", nargs="+", default=list(PARTS), choices=list(PARTS))
    ap.add_argument("--resume", action="store_true",
                    help="keep the solves a previous run already wrote")
    ap.add_argument("--out", default=OUT,
                    help="output directory (default: runs/comparison; "
                         "data/comparison replaces the shipped files)")
    args = ap.parse_args()
    Q.run_parts(PARTS, args.part, os.path.abspath(args.out), args.resume)


if __name__ == "__main__":
    main()
