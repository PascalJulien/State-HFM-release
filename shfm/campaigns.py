"""Building and re-running the Bench-MR campaigns.

The figures that compare this planner with sampling-based ones read result
files rather than re-running the planners, and so does this package: the five
campaigns ship in ``data/campaigns/``. Four of the five planners they hold
are anytime, so a re-run explores a different tree and returns different paths
on a differently loaded machine; RRT stops at its first solution.

This module holds the code that produced those files, so that the campaigns
can be re-run. ``scripts/verify_campaigns.py`` checks it: with the settings of
``data/campaigns/settings.json``, RRT reproduces its runs, their normalised
curvature aside (``external/NOTICE.md``).

Every per-scene setting is read from that file.
"""
import json
import os
import sys
import tempfile

# Re-exported from shfm.benchmark.
from .benchmark import PLANNERS, SMOOTHERS, STEER_FUNCTION

__all__ = ["PLANNERS", "SMOOTHERS", "STEER_FUNCTION", "settings", "n_runs",
           "make_mpb", "regenerate", "strip_paths"]

_BENCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "external", "bench-mr")


def _mpb_module():
    """Bench-MR's own MPB, pointed at the vendored binary."""
    from .metrics import bindings
    bindings()
    path = os.path.join(_BENCH, "python")
    if path not in sys.path:
        sys.path.insert(0, path)
    import mpb
    mpb.MPB_BINARY = os.path.join(_BENCH, "bin", "benchmark")
    mpb.MPB_BINARY_DIR = os.path.join(_BENCH, "bin")
    # MPB resolves the template path relative to the working directory, and the
    # default is frozen at import. Rebind it so the module works from anywhere.
    template = os.path.join(_BENCH, "bin", "benchmark_template.json")
    for fn in (mpb.MPB.__init__, mpb.MPB.get_config):
        target = getattr(fn, "__func__", fn)
        if target.__defaults__:
            target.__defaults__ = tuple(
                template if isinstance(d, str) and d.endswith("benchmark_template.json")
                else d for d in target.__defaults__)
    return mpb


SETTINGS_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "data", "campaigns", "settings.json")
"""What each campaign was launched with, kept apart from what it returned.

A Bench-MR result file carries its configuration twice and neither copy can be
re-run as it stands. The block at the top holds the absolute paths of the
machine that produced it. The block inside each run is the state *after* that
run, with the start, the goal and the vehicle polygon resolved and the
environment seed already advanced: re-running from it shifts every
procedurally generated map by one instance, and the scene is no longer the same
scene.

This file is the top block of each campaign with the paths reduced to file
names and the map images renamed after those of ``data/maps/``, which lets a
campaign be re-run when its results are not at hand.
"""


def settings(scene):
    """The configuration a campaign was launched with, under the file names
    of this repository."""
    with open(SETTINGS_FILE, encoding="utf-8") as f:
        return json.loads(json.dumps(json.load(f)[scene]["settings"]))


def n_runs(scene):
    """How many runs a campaign holds."""
    with open(SETTINGS_FILE, encoding="utf-8") as f:
        return json.load(f)[scene]["runs"]


MAPS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "data", "maps")
"""The scene images. Bench-MR resamples each one to the size the
configuration asks for."""

FOOTPRINTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "data", "footprints")
"""The three SVG envelopes the campaigns passed to the planners.

Bench-MR sizes them from their half
width and moves the reference point to the rear axle of a 3.6490 m vehicle, so
the shape that reaches the collision checker is 4.133 m by 0.700 m on the
navigation scenes and 3.306 m by 0.560 m on the docking one. Neither matches
the vehicle this package declares; they are shipped unchanged, because
changing them would change the results.
"""


def make_mpb(scene, planner, map_path=None, footprint=None):
    """One Bench-MR job, configured as its campaign was configured.

    Everything comes from ``settings.json``; only the planner and the two file
    paths are substituted, so the scene points at the copies that live here.
    """
    mpb = _mpb_module()
    config = settings(scene)
    for key in config["benchmark"]["planning"]:
        config["benchmark"]["planning"][key] = (key == planner)
    # settings.json names the scene image without its path: resolve it against
    # the maps shipped here so the campaign runs wherever the folder is copied.
    image = config["env"]["grid"].get("image", {})
    if map_path is None and image.get("source"):
        candidate = os.path.join(MAPS, os.path.basename(image["source"]))
        map_path = candidate if os.path.exists(candidate) else None
    if map_path is not None:
        config["env"]["grid"]["image"]["source"] = os.path.abspath(map_path)
    if footprint is None:
        # Point at the vendored copy of whichever envelope the campaign used.
        footprint = os.path.join(
            FOOTPRINTS,
            os.path.basename(config["env"]["collision"]["robot_shape_source"]))
    if os.path.exists(footprint):
        config["env"]["collision"]["robot_shape_source"] = os.path.abspath(footprint)
    job = mpb.MPB(config_file=os.path.join(_BENCH, "bin", "benchmark_template.json"))
    job.config = config
    job._update_pss()
    return job


def regenerate(scene, runs=None, planners=None, workdir=None, silence=True):
    """Re-run a campaign and merge it into one result file.

    Returns the path of the merged file. Use it to check the pipeline, not to
    replace :func:`shfm.benchmark.load_campaign`: the anytime planners will not
    return the same paths on a machine loaded differently.
    """
    mpb = _mpb_module()
    runs = runs or n_runs(scene)
    workdir = workdir or tempfile.mkdtemp(prefix="shfm-campaign-")
    os.makedirs(workdir, exist_ok=True)
    for name in ("polygon_mazes", "sbpl_mprim", "image_mazes"):
        src = os.path.join(_BENCH, "bin", name)
        dst = os.path.join(workdir, name)
        if os.path.isdir(src) and not os.path.exists(dst):
            os.symlink(src, dst)

    pool = mpb.MultipleMPB()
    for planner in (planners or PLANNERS):
        pool.benchmarks.append(make_mpb(scene, planner))
    cwd = os.getcwd()
    os.chdir(workdir)
    try:
        pool.run_parallel(runs=runs, id=scene, show_plot=False)
        merged = os.path.join(workdir, "%s.json" % scene)
        pool.merge(merged, silence=silence)
    finally:
        os.chdir(cwd)
    return merged


def strip_paths(data):
    """Replace the absolute paths Bench-MR writes into a result by file names.

    The map image, the vehicle outline and the log file are named, with the
    directory of the machine that ran the benchmark, in the settings of the
    result and of every run, and in the ``generator`` field of every run
    environment. Nothing reads them back but their names: the map of a run is
    stored in the run itself. Modifies ``data`` in place and returns it.
    """
    def fix(settings):
        if not settings:
            return
        bench = settings.get("benchmark") or {}
        if bench.get("log_file"):
            bench["log_file"] = os.path.basename(bench["log_file"])
        env = settings.get("env") or {}
        shape = env.get("collision") or {}
        if shape.get("robot_shape_source"):
            shape["robot_shape_source"] = os.path.basename(shape["robot_shape_source"])
        image = (env.get("grid") or {}).get("image") or {}
        if image.get("source"):
            image["source"] = os.path.basename(image["source"])

    fix(data.get("settings"))
    for run in data.get("runs", []):
        fix(run.get("settings"))
        env = run.get("environment") or {}
        gen = env.get("generator")
        if isinstance(gen, str) and gen.startswith("image: "):
            env["generator"] = "image: " + os.path.basename(gen[len("image: "):])
    return data
