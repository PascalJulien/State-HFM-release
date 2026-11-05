"""The comparison plates, drawn by Bench-MR's own plotting tools.

The plates that compare this planner with sampling-based ones are drawn by the
code that draws the latter: :func:`trajectory.visualize` for the trajectory
plates and :func:`plot_stats.plot_planner_stats` for the metric violins, with
our own paths injected through ``additional_path`` and ``custom_stats``.

One panel is not a Bench-MR metric: the Approach Heading Error
(:func:`shfm.metrics.approach_heading_error`). Its title is added to Bench-MR's
table of names when the tools are imported, and the values of the sampling
planners, which the campaign files do not hold, are measured on their
trajectories (:func:`campaign_ahe`).

The keyword arguments below are the ones the plates use. They are kept as
defaults, so that a call only needs the scene and the metric.
"""
import json
import os
import sys

import numpy as np

from . import metrics as _m

__all__ = ["OUR_PLANNERS", "custom_stats", "campaign_ahe", "trajectories",
           "metric_violin", "metric_plate", "save_stats", "load_stats"]

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PY = os.path.join(_REPO, "external", "bench-mr", "python")

OUR_PLANNERS = ("SHFM_2WS", "SHFM_4WS")
"""Names under which our own paths enter the metric plots, as keys of
:func:`custom_stats`. The trajectory plates name them ``shfm2ws`` and
``shfm4ws``, the names ``ignore_planners`` takes.

``plot_stats`` gives the last two series a fixed blue and red,
so the order of this tuple is what puts 2WS in blue and 4WS in red.
"""

_AHE = "approach_heading_error"

_STAT_KEYS = ("path_length", "aol", "normalized_curvature", "max_curvature",
              "max_clearing_distance", "mean_clearing_distance",
              "median_clearing_distance", "min_clearing_distance", _AHE)


def _tools():
    """Import Bench-MR's plotting modules, once the extension is loadable."""
    _m.bindings()                       # pins the C++ runtime before matplotlib
    # viz_mpb lives beside the figure generators, not in Bench-MR.
    generators = os.path.join(_REPO, "generators")
    for path in (_PY, generators):
        if path not in sys.path:
            sys.path.insert(0, path)
    # Bench-MR ends its plotting with plt.show(). Under a file-writing backend
    # that warns, once per call, and the warning carries the absolute path of
    # the vendored file that raised it, which then sits in the notebook output.
    import warnings
    warnings.filterwarnings("ignore", category=UserWarning,
                            message="Matplotlib is currently using agg")
    from plot_stats import plot_planner_stats
    from trajectory import visualize
    import definitions
    import viz_mpb
    definitions.stat_names.setdefault(_AHE, "Approach Heading Error")
    from .figures import FIGURE_DIR
    # viz_mpb rebuilds every save path as OUT_DIR/basename, so this is what
    # decides how the "Saved ..." lines read in the notebooks. Relative, for the
    # reason given on _short below. Resolved against the working directory in
    # force when the tools are first imported, which for the notebooks is
    # notebooks/ and does not move afterwards.
    viz_mpb.set_output(out_dir=_short(FIGURE_DIR))
    return visualize, plot_planner_stats, viz_mpb


def save_stats(stats, path):
    """Write the statistics of our own paths, so another notebook can read them.

    They are plain lists of numbers keyed by planner and by metric, which is
    what :func:`custom_stats` returns and what the plotting tools take back.
    """
    with open(path, "w") as f:
        json.dump(stats, f)
    return path


def load_stats(path):
    """Read back what :func:`save_stats` wrote."""
    with open(path) as f:
        return json.load(f)


def _per_run(value, n):
    """Accept one value for every run, or one per run."""
    if isinstance(value, (list, tuple)) and len(value) == n:
        return list(value)
    return [value] * n


def custom_stats(paths, env, goal, raw=None):
    """Metrics of our own paths, in the shape ``plot_planner_stats`` expects.

    ``paths`` maps a name from :data:`OUR_PLANNERS` to the list of paths, one
    per run of the campaign, in the order they are driven. ``env`` and ``goal``
    are either one value for the whole campaign or one per run; the generated
    scenes need the second form, since every instance has its own map and its
    own endpoints. The metrics are the same Bench-MR implementations the
    sampling planners are measured with.

    ``raw`` optionally carries the same paths as the solver produced them, from
    the goal back to the start. Bench-MR forms its curvature triples from one
    end, so the traversal order shifts the curvature by a few percent. The
    plates the notebooks draw measure curvature on the raw output, so when
    ``raw`` is given the two curvature columns come from it. The AHE always
    comes from the driven path, since it looks at where it ends, over the
    length of the vehicle of ``env``.
    """
    out = {}
    for name, runs in paths.items():
        others = (raw or {}).get(name)
        envs, goals = _per_run(env, len(runs)), _per_run(goal, len(runs))
        row = {k: [] for k in _STAT_KEYS}
        for i, p in enumerate(runs):
            p = np.asarray(p, dtype=float)
            curve = np.asarray(others[i], dtype=float) if others else p
            e, g = envs[i], goals[i]
            with _m.quiet():
                mx, norm = _m.curvature(curve)
                clear = _m.clearing_distances(p, e.walls)
                row["path_length"].append(_m.path_length(p))
                row["aol"].append(_m.aol(p))
                row["normalized_curvature"].append(norm)
                row["max_curvature"].append(mx)
                row["max_clearing_distance"].append(float(np.max(clear)))
                row["mean_clearing_distance"].append(float(np.mean(clear)))
                row["median_clearing_distance"].append(float(np.median(clear)))
                row["min_clearing_distance"].append(float(np.min(clear)))
                row[_AHE].append(_m.approach_heading_error(
                    p, g[2], e.vehicle.length * e.zoom))
        out[name] = row
    return out


def campaign_ahe(json_file, zoom, vehicle=None):
    """AHE of the sampling planners of a campaign, in the shape of
    :func:`custom_stats`: ``{planner: {"approach_heading_error": [...]}}``.

    The campaign files do not hold it. It is measured on the stored
    trajectories, over the length of ``vehicle`` (the default vehicle when
    None) at ``zoom``, the zoom the campaign ran at, and for the runs that
    reached the goal only: a run that stopped short has no approach to
    measure. A planner that never reached it is left out.
    """
    from . import DEFAULT_VEHICLE
    from .benchmark import endpoints_of, reached_goal
    with open(json_file) as f:
        data = json.load(f)
    window = (vehicle or DEFAULT_VEHICLE).length * zoom
    out = {}
    for run in data["runs"]:
        goal = endpoints_of(run)[1]
        for name, plan in run["plans"].items():
            if not plan or not reached_goal(plan) or not plan.get("trajectory"):
                continue
            out.setdefault(name, {_AHE: []})[_AHE].append(
                _m.approach_heading_error(plan["trajectory"], goal[2], window))
    return out


def _with_campaign_ahe(json_file, stats, metrics, zoom, ignore_planners=""):
    """``stats``, plus the AHE of the sampling planners when a panel shows it.

    ``plot_planner_stats`` appends the values of ``custom_stats`` to those it
    reads from the campaign, so the sampling planners keep their place and
    their colour.
    """
    if _AHE not in [m.strip().lower() for m in metrics.split(",")]:
        return stats
    if zoom is None:
        raise ValueError("the %s panel needs the zoom of the campaign, which "
                         "sizes the vehicle in grid units" % _AHE)
    ignored = {p.strip().lower() for p in ignore_planners.split(",")}
    out = {k: v for k, v in campaign_ahe(json_file, zoom).items()
           if k.lower() not in ignored}
    out.update(stats)
    return out


def _unit_labels(zoom):
    """Bench-MR's ``plot_env``, with the metre equivalence added to the axes.

    ``plot_env`` is wrapped rather than the figure relabelled afterwards, and
    the labels are set without a font size, so they come out at the size
    matplotlib is configured for. The wrapper is installed on the two modules
    that hold a reference to ``plot_env``, and removed again once the figure is
    drawn.
    """
    import contextlib
    import plot_env as _pe
    import trajectory as _t
    import matplotlib.pyplot as plt

    @contextlib.contextmanager
    def patched():
        original = _pe.plot_env
        note = "1 unit = %.3g m" % (1.0 / float(zoom))

        def wrapper(*args, **kwargs):
            out = original(*args, **kwargs)
            ax = plt.gca()
            ax.set_xlabel("$x$ (units ; %s)" % note)
            ax.set_ylabel("$y$ (units ; %s)" % note)
            return out

        _pe.plot_env = wrapper
        _t.plot_env = wrapper
        try:
            yield
        finally:
            _pe.plot_env = original
            _t.plot_env = original

    return patched()


def _short(path):
    """The same file, named relatively, when that is shorter.

    Bench-MR echoes the name of every plate it writes, and those lines are
    stored in the notebooks; a relative name keeps them free of the directory
    of the machine that ran them.
    """
    if path is None:
        return None
    try:
        rel = os.path.relpath(path)
    except ValueError:                      # different drive, on Windows
        return path
    return rel if len(rel) < len(path) else path


def trajectories(json_file, ours=None, every_nth=50, run_id="0",
                 ignore_planners="", save_file=None, set_title=False,
                 show_legend=True, max_plots_per_line=1, zoom=None,
                 dpi=200, **kwargs):
    """The trajectory plate: every planner's path on one scene.

    ``ours`` is ``[paths_2ws, paths_4ws]``, each a list of ``(n, 3)`` arrays,
    one per run, which is what goes into
    ``additional_path``.
    """
    import contextlib
    import matplotlib.pyplot as plt
    visualize, _, viz = _tools()
    # visualize draws on the current figure: without closing it, two calls in
    # a row would stack one plate on top of the other.
    plt.close("all")
    # Bench-MR echoes the file it reads, a temporary path, into the notebook
    # outputs; nothing else depends on the flag.
    kwargs.setdefault("silence", True)
    with (_unit_labels(zoom) if zoom is not None else contextlib.nullcontext()):
        visualize(json_file, additional_path=ours,
                  plot_every_nth_polygon=every_nth,
                  draw_start_goal_thetas=True, run_id=run_id,
                  ignore_planners=ignore_planners, save_file=None,
                  set_title=set_title, show_legend=show_legend,
                  max_plots_per_line=max_plots_per_line, headless=True, **kwargs)
    fig = plt.gcf()
    if save_file is not None:
        save_file = _short(save_file)
        fig.savefig(save_file, dpi=dpi, bbox_inches="tight")
        stem, ext = os.path.splitext(save_file)
        for leg in list(fig.legends):
            leg.remove()
        fig.savefig(stem + "_nolegend" + ext, dpi=dpi, bbox_inches="tight")
    return fig


def metric_violin(json_file, stats, metric, save_file=None, zoom=None, **kwargs):
    """One metric, one violin per planner.

    ``zoom``, the zoom the campaign ran at, is needed for the AHE only; see
    :func:`campaign_ahe`. Writes ``save_file`` and a copy suffixed
    ``_nolegend`` under :data:`shfm.figures.FIGURE_DIR`: only the base name of
    ``save_file`` is kept.
    """
    import matplotlib.pyplot as plt
    _, plot_planner_stats, _viz = _tools()
    # plot_planner_stats draws on the current figure, like visualize. Without
    # this, a second call lands on top of the first and the plate ends up with
    # two metrics on one pair of axes and a doubled legend.
    plt.close("all")
    opts = dict(silence=True, metrics=metric, max_plots_per_line=1,
                combine_views=True, plot_violins=True, fig_width=10,
                fig_height=6, ticks_rotation=0, save_file=_short(save_file))
    opts.update(kwargs)
    opts["custom_stats"] = _with_campaign_ahe(json_file, stats, opts["metrics"], zoom,
                                              opts.get("ignore_planners", ""))
    return plot_planner_stats(json_file, **opts)


def metric_plate(json_file, stats, save_file=None, zoom=None,
                 metrics=("path_length", "aol", "max_curvature",
                          "min_clearing_distance", "approach_heading_error"),
                 **kwargs):
    """The summary plate, one panel per metric, two per row.

    A panel alone on the last row is centred under the row above it. ``zoom``,
    the zoom the campaign ran at, is needed for the AHE; see
    :func:`campaign_ahe`. Like ``plot_planner_stats``, writes ``save_file``
    and a copy suffixed ``_nolegend``, both under
    :data:`shfm.figures.FIGURE_DIR`: only the base name of ``save_file`` is
    kept.
    """
    import matplotlib.pyplot as plt
    _, plot_planner_stats, viz = _tools()
    plt.close("all")
    opts = dict(silence=True, metrics=",".join(metrics),
                max_plots_per_line=2, fig_width=6, fig_height=4,
                combine_views=True, plot_violins=True, ticks_rotation=25,
                show_legend=False, dpi=200)
    opts.update(kwargs)
    opts["custom_stats"] = _with_campaign_ahe(json_file, stats, opts["metrics"], zoom,
                                              opts.get("ignore_planners", ""))
    # Drawn without saving or showing, so that the last panel can be moved
    # first; both are done below, as plot_planner_stats would.
    show = not opts.pop("headless", False)
    plot_planner_stats(json_file, save_file=None, headless=True, **opts)
    fig = plt.gcf()
    axes, per_line = fig.axes, opts["max_plots_per_line"]
    if len(axes) > per_line and len(axes) % per_line == 1:
        left = axes[-1 - per_line].get_position()
        right = axes[-2].get_position()
        alone = axes[-1].get_position()
        centre = (left.x0 + right.x1) / 2
        axes[-1].set_position([centre - alone.width / 2, alone.y0,
                               alone.width, alone.height])
    if save_file is not None:
        save_file = viz.redirect(_short(save_file))
        stem, ext = os.path.splitext(save_file)
        fig.savefig(save_file, dpi=opts["dpi"], bbox_inches="tight")
        print("Saved %s." % save_file)
        for ax in axes:
            if ax.get_legend() is not None:
                ax.get_legend().remove()
        fig.savefig(stem + "_nolegend" + ext, dpi=opts["dpi"], bbox_inches="tight")
        print("Saved %s." % (stem + "_nolegend" + ext))
    if show:
        plt.show()
