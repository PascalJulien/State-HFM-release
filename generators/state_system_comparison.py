#!/usr/bin/env python3
"""Paths planned without and with the state system, on the same scene.

Without the state system, the planner is given the eight extreme 4WS
controls, both axles at 1.05 rad, forward and backward, all at unit speed and
in a single state: crab motion is available everywhere at no extra cost. With
it, the same kinematics are split into the four modes of :mod:`shfm.states`,
navigation with the rear axle locked and manoeuvring with both axles, forward
and backward, with the speed coefficients ``nu_fwd = 3``, ``nu_2ws = 1.5``,
``nu_4ws = 0.5``, ``gamma_rev = 0.5``, and every change of mode priced at one
unit.

Free switching is expressed by the single state of
:func:`shfm.kinematics.control_set_unstructured`, not by two states joined by
a zero transition matrix: the solver squares and inverts that matrix, and a
zero entry leaves part of the value function at infinity without an error.

Both solves share the scene of ``data/state_system/bays.json``, a 54 x 54
grid at one metre per cell, with 100 orientations, the obstacle cost and its
clearance capped at the vehicle length, the start (50, 30, pi/2) and the goal
(22, 45, 3 pi/2), both in the first mode. The paths are drawn by Bench-MR's
own trajectory plot, the footprint every 40 (without) or 50 (with) points.

Writes no_state.pdf and with_state.pdf. Needs a CUDA device and the Bench-MR
metric bindings (``scripts/setup.sh``), which the plotting tools of
``external/bench-mr/python`` import.
"""
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)
OUT = os.environ.get("SHFM_FIGURE_DIR", os.path.join(_REPO, "figures"))
os.makedirs(OUT, exist_ok=True)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from shfm.environments import from_benchmark_run
from shfm.kinematics import control_set, control_set_unstructured
from shfm.plates import trajectories
from shfm.solver import Planner

SCENE = os.path.join(_REPO, "data", "state_system", "bays.json")
ZOOM = 1.0                 # one grid unit per metre
N_THETA = 100
STEERING = 1.05            # saturation of both axles, rad
COEFFICIENTS = dict(nu_fwd=3.0, nu_2ws=1.5, nu_4ws=0.5, gamma_rev=0.5)
SWITCH_COST = 1.0          # every change of mode
FM2 = 1.0                  # weight of the clearance term
EVERY_NTH = {"no_state": 40, "with_state": 50}


def unstructured(theta, wheelbase_u, xp):
    return control_set_unstructured(theta, wheelbase_u, delta_f_max=STEERING,
                                    delta_r_max=STEERING, xp=xp)


def four_modes(theta, wheelbase_u, xp):
    return control_set(theta, wheelbase_u, delta_f_max=STEERING,
                       delta_r_max=STEERING, xp=xp, **COEFFICIENTS)


def solve(env, start, goal):
    """Geodesics without and with the state system, as the solver returns
    them (goal to start), ``(n, 3)``."""
    seed, tip = list(start) + [0], list(goal) + [0]
    common = dict(obstacle_cost=True, saturate=True, fm2=FM2)
    without = Planner(env, controls=unstructured, **common)
    with_states = Planner(env, controls=four_modes,
                          transitions=SWITCH_COST * (np.ones((4, 4)) - np.eye(4)),
                          **common)
    paths = {}
    for name, planner in (("no_state", without), ("with_state", with_states)):
        result = planner.solve(seed, [tip])
        paths[name] = result.raw_path(0)[:, :3]
        stops = result.hfm_out.get("geodesic_stopping_criteria")
        print("%-10s %4d points, length %.1f, stop %s"
              % (name, len(paths[name]),
                 np.hypot(*np.diff(paths[name][:, :2], axis=0).T).sum(),
                 stops[0] if stops is not None else "?"))
    return paths


def draw(path, name):
    """Bench-MR trajectory plate of one path, without legend.

    The plate draws our paths in two slots, blue and red; the path without
    states takes the red one, the path with states the blue one.
    """
    ignore = "shfm2ws" if name == "no_state" else "shfm4ws"
    # Bench-MR echoes the name of the file it reads; a relative name keeps
    # the home directory out of that line.
    scene = os.path.relpath(SCENE)
    scene = scene if len(scene) < len(SCENE) else SCENE
    fig = trajectories(scene, ours=[[path], [path]], every_nth=EVERY_NTH[name],
                       run_id="0", ignore_planners=ignore, save_file=None,
                       set_title=False, show_legend=False)
    # The plate reserves room on the right for a figure legend; without the
    # legend, the default subplot layout is restored.
    for legend in list(fig.legends):
        legend.remove()
    fig.subplots_adjust(right=plt.rcParams["figure.subplot.right"])
    out = os.path.join(OUT, name + ".pdf")
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(out)


if __name__ == "__main__":
    with open(SCENE) as f:
        run = json.load(f)["runs"][0]
    env = from_benchmark_run(run, zoom=ZOOM, n_theta=N_THETA)
    paths = solve(env, run["environment"]["start"], run["environment"]["goal"])
    for name in ("no_state", "with_state"):
        draw(paths[name], name)
