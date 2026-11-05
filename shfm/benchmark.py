"""Running the Bench-MR sampling-based planners on the same problem.

The footprint settings below make both sides plan for the same vehicle when
``footprint_svg`` is the envelope that :meth:`shfm.vehicle.Vehicle.to_svg`
writes for it (``data/forklift.svg`` for the default vehicle): Bench-MR scales
it by the grid zoom and keeps its reference point, the centre of the rear
axle, at the origin, which is the point the steering functions integrate.
``make_config`` reads only the wheelbase from ``vehicle``.

The campaigns shipped in ``data/campaigns/`` predate these settings. They were
run with the envelopes of ``data/footprints/``, which do not match
:data:`shfm.vehicle.DEFAULT_VEHICLE`; see :data:`shfm.campaigns.FOOTPRINTS`
and ``data/footprints/README.md``.

Metrics are not recomputed here. Both sides are measured by
:mod:`shfm.metrics`, which calls the Bench-MR implementation.
"""
import gzip
import json
import os
import shutil
import subprocess
import tempfile

import numpy as np

__all__ = ["BENCH_ROOT", "BINARY", "PLANNERS", "make_config", "run",
           "paths_by_planner", "endpoints_of", "reached_goal",
           "SMOOTHERS", "STEER_FUNCTION", "TEMPLATE",
           "CAMPAIGN_DIR", "CAMPAIGNS", "load_campaign", "campaign_file",
           "campaign_path"]

BENCH_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "external", "bench-mr")
BINARY = os.path.join(BENCH_ROOT, "bin", "benchmark")
TEMPLATE = os.path.join(BENCH_ROOT, "bin", "benchmark_template.json")

PLANNERS = ["rrt", "rrt_star", "informed_rrt_star", "sst", "spars"]
"""The five sampling-based planners this one is compared against.

Re-exported by :mod:`shfm.campaigns`.
"""

SMOOTHERS = ("grips", "ompl_bspline", "ompl_shortcut", "ompl_simplify_max")
"""Smoothers, as the campaigns enabled them."""

STEER_FUNCTION = "reeds_shepp"
"""Steering function the campaigns were run with."""


def make_config(map_path, start, goal, vehicle, zoom, footprint_svg,
                planners=None, runs=20, planning_time=1.0,
                steering=STEER_FUNCTION, desired_size=None, log_file="results",
                delta_max=1.05, goal_radius=0.01, ompl_seed=5,
                turning_radius_delta=0.5,
                smoothers=SMOOTHERS,
                generator="image",
                grid_size=(100, 100), seed=1, corridor_radius=5.0,
                branches=50, obstacle_ratio=0.1):
    """Build a Bench-MR configuration for one scene.

    ``start`` and ``goal`` are ``(x, y, theta)`` in grid units, the same frame
    the planner uses.

    With the default ``generator="image"`` the scene is read from ``map_path``.
    The two other generators build it procedurally instead, and ``map_path``,
    ``start`` and ``goal`` are then ignored:

    ``"corridor"``
        Interconnected corridors of half-width ``corridor_radius``, grown from
        the centre towards ``branches`` random targets, each joined to the
        closest existing node. Start and goal are the two nodes furthest apart,
        so they are chosen by the generator and must be read back from the run
        with :func:`endpoints_of`.
    ``"random"``
        A fraction ``obstacle_ratio`` of the cells occupied at random.

    ``seed`` makes either of them reproducible.
    """
    if not os.path.exists(TEMPLATE):
        raise RuntimeError(
            "Bench-MR settings template not found at %s. Build Bench-MR with "
            "./scripts/setup.sh, which writes it." % TEMPLATE)
    with open(TEMPLATE) as f:
        cfg = json.load(f)["settings"]

    bench = cfg["benchmark"]
    for name in bench["planning"]:
        bench["planning"][name] = name in (planners or PLANNERS)
    bench["runs"] = int(runs)
    bench["control_planners_on"] = False
    bench["steer_functions"] = [{"reeds_shepp": 0, "4ws_state_space": 10}[steering]]
    bench["log_file"] = log_file
    # The smoothers do not change the base statistics, which are measured on
    # the planner's own path, but they are part of the run and of its timing.
    for key in bench["smoothing"]:
        bench["smoothing"][key] = key in smoothers
    cfg["ompl"]["seed"] = int(ompl_seed)

    if generator not in ("image", "corridor", "random"):
        raise ValueError("unknown generator %r" % (generator,))

    grid = cfg["env"]["grid"]
    grid["generator"] = generator
    grid["seed"] = int(seed)
    grid["width"], grid["height"] = int(grid_size[0]), int(grid_size[1])
    grid["corridor"].update(radius=float(corridor_radius),
                            branches=int(branches))
    grid["random"].update(obstacle_ratio=float(obstacle_ratio))
    if generator == "image":
        if map_path is None:
            raise ValueError('generator="image" needs a map_path')
        grid["image"].update(
            source=os.path.abspath(map_path), occupancy_threshold=0.5,
            desired_width=int(desired_size[0]) if desired_size else 0,
            desired_height=int(desired_size[1]) if desired_size else 0)
        cfg["env"]["start"] = dict(x=float(start[0]), y=float(start[1]),
                                   theta=float(start[2]))
        cfg["env"]["goal"] = dict(x=float(goal[0]), y=float(goal[1]),
                                  theta=float(goal[2]))

    collision = cfg["env"]["collision"]
    collision["collision_model"] = 1                       # polygon
    collision["robot_shape_source"] = os.path.abspath(footprint_svg)
    collision["robot_shape_scale"] = float(zoom)
    collision["robot_reference_x"] = 0.0                   # already at the origin
    collision["robot_reference_y"] = 0.0

    wheelbase_u = vehicle.wheelbase * zoom
    cfg["steer"]["steering_type"] = 0
    cfg["steer"]["wheel_base"] = float(wheelbase_u)
    cfg["steer"]["delta_max"] = float(delta_max)
    # The turning radius is sized on its own angle, 0.5 rad by default, as
    # the navigation campaigns were; the docking campaign used 1.05 rad.
    # settings.json holds the radius each campaign was given.
    cfg["steer"]["car_turning_radius"] = float(
        wheelbase_u / np.tan(turning_radius_delta))

    cfg["max_planning_time"] = float(planning_time)
    cfg["exact_goal_radius"] = float(goal_radius)
    return {"settings": cfg}


def run(config, workdir=None, timeout=3600):
    """Run the benchmark binary on ``config`` and return the parsed results.

    The binary resolves a few resource directories relative to its working
    directory and writes scratch files there, so it is run in a directory of
    its own with those directories linked in.
    """
    if not os.path.exists(BINARY):
        raise RuntimeError(
            "Bench-MR binary not found at %s. Build it with ./scripts/setup.sh"
            % BINARY)

    created = workdir is None
    workdir = workdir or tempfile.mkdtemp(prefix="shfm-bench-")
    os.makedirs(workdir, exist_ok=True)
    for name in ("polygon_mazes", "sbpl_mprim", "image_mazes"):
        src = os.path.join(BENCH_ROOT, "bin", name)
        dst = os.path.join(workdir, name)
        if os.path.isdir(src) and not os.path.exists(dst):
            os.symlink(src, dst)

    log_name = config["settings"]["benchmark"]["log_file"] or "results"
    cfg_path = os.path.join(workdir, "config.json")
    with open(cfg_path, "w") as f:
        json.dump(config, f, indent=1)

    proc = subprocess.run([BINARY, "config.json"], cwd=workdir, timeout=timeout,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = os.path.join(workdir, log_name)
    if not os.path.exists(out):
        tail = proc.stdout.decode("utf-8", "replace")[-2000:]
        raise RuntimeError("benchmark produced no results file.\n" + tail)
    with open(out) as f:
        results = json.load(f)
    if created:
        shutil.rmtree(workdir, ignore_errors=True)
    return results


def paths_by_planner(results, run_index=0):
    """``{planner: path}`` for one run, paths as ``(n, 3)`` arrays."""
    out = {}
    for name, plan in results["runs"][run_index]["plans"].items():
        traj = (plan or {}).get("trajectory")
        if not traj:
            continue
        out[name] = np.asarray(traj, dtype=float)[:, :3]
    return out


def endpoints_of(run):
    """``(start, goal)`` as ``(x, y, theta)``, read back from one run.

    Needed for the procedural generators, which choose the endpoints
    themselves rather than taking the ones given in the configuration.
    """
    env = run["environment"]
    return (tuple(float(v) for v in env["start"]),
            tuple(float(v) for v in env["goal"]))


def reached_goal(plan):
    """True when this plan terminates at the requested pose.

    Under a time budget a sampling planner returns the path found so far, which
    may stop well short. Bench-MR records that as ``exact_goal_path``.
    Averaging metrics over runs that did not reach the goal mixes two different
    quantities and inflates the spread.
    """
    stats = (plan or {}).get("stats") or {}
    return bool(stats.get("exact_goal_path"))


CAMPAIGN_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "data", "campaigns")

CAMPAIGNS = ("two_rooms", "warehouse", "procedural", "narrow_passage",
             "docking")
"""The five benchmark campaigns, one file per scene, in ``data/campaigns/``.

A campaign is one run of the five sampling-based planners over the same scene,
twenty times over, except ``narrow_passage``, which holds two runs, that
figure being a single trajectory plate rather than a distribution. Read the
count from ``shfm.campaigns.n_runs`` rather than assuming it. The files hold
the statistics of every run and the trajectory of every plan that found a
path; the AHE of the sampling planners is measured on those trajectories.

They ship with the repository rather than being produced on first use,
because four of the five planners in them are anytime: they explore for as
long as their time budget allows, so they return a different tree on a
differently loaded machine. At the docking scene, RRT* and Informed-RRT*
often stop well before their budget (median 0.37 s out of 15 s). RRT stops at its first solution and reproduces,
its normalised curvature aside (``external/NOTICE.md``).
Re-running gives the same distribution, not the same paths.
``scripts/regenerate_campaigns.py`` re-runs them into ``runs/``,
and ``scripts/verify_campaigns.py`` rebuilds one planner at a time and reports
how far it agrees with the shipped runs.
"""


def load_campaign(name):
    """One campaign, parsed."""
    path = campaign_path(name)
    with gzip.open(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def campaign_path(name):
    """Where a campaign lives."""
    if name not in CAMPAIGNS:
        raise ValueError("unknown campaign %r, expected one of %s"
                         % (name, ", ".join(CAMPAIGNS)))
    return os.path.join(CAMPAIGN_DIR, name + ".json.gz")


def campaign_file(name, cache_dir=None):
    """Path to a decompressed campaign, unpacking it on first use.

    Bench-MR's plotting tools take a file name, so the gzipped copy is expanded
    once into ``cache_dir``. The default is a folder of the system temporary
    directory that belongs to the current user, so that two users of one
    machine do not write over each other's copy.
    """
    packed = campaign_path(name)
    if cache_dir is None:
        uid = os.getuid() if hasattr(os, "getuid") else 0
        cache_dir = os.path.join(tempfile.gettempdir(), "shfm-%d" % uid)
    os.makedirs(cache_dir, exist_ok=True)
    out = os.path.join(cache_dir, name + ".json")
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(packed):
        with gzip.open(packed, "rt", encoding="utf-8") as f:
            data = f.read()
        # Written aside and moved in place, so that a reader running at the
        # same time never sees half a file.
        tmp = out + ".%d.tmp" % os.getpid()
        with open(tmp, "w") as f:
            f.write(data)
        os.replace(tmp, out)
    return out
