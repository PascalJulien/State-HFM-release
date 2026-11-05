"""Path metrics, evaluated by Bench-MR.

Every metric here but one is computed by the Bench-MR implementation, through
its Python bindings, rather than reimplemented: this planner is compared
against sampling-based planners measured by that same code. The exception is
the Approach Heading Error, :func:`approach_heading_error`, which Bench-MR does
not define; it is computed here, the same way for every planner.

The angular and euclidean deviations at the goal are Bench-MR's. They remain
available, and the published data keep them, but the results no longer use
them: the Approach Heading Error took their place.

Build the bindings once with ``./scripts/setup.sh``.
"""
import contextlib
import ctypes
import glob
import os
import sys

import numpy as np

__all__ = ["bindings", "quiet", "path_length", "curvature", "max_curvature",
           "normalized_curvature", "clearing_distances", "aol",
           "approach_heading_error",
           "angular_deviation_at_goal", "euclidean_deviation_at_goal",
           "swept_area",
           "summarize"]

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_BINDINGS_DIR = os.path.join(_REPO, "external", "bench-mr", "python")
_mb = None


def _preload_libstdcxx():
    """Load the interpreter's own libstdc++ before the Bench-MR extension.

    The extension is linked against the system C++ runtime. Importing it first
    pins that runtime for the whole process, after which anything built against
    a newer one, such as matplotlib in a conda environment, fails to import.
    Loading the newer library first avoids this.
    """
    for pattern in (os.path.join(sys.prefix, "lib", "libstdc++.so.6"),
                    os.path.join(sys.prefix, "lib", "libstdc++.so.6.*")):
        for path in sorted(glob.glob(pattern)):
            try:
                ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
                return path
            except OSError:
                continue
    return None


def bindings():
    """The Bench-MR metrics module, imported on first use."""
    global _mb
    if _mb is None:
        _preload_libstdcxx()
        if _BINDINGS_DIR not in sys.path:
            sys.path.insert(0, _BINDINGS_DIR)
        try:
            import metrics_bindings
        except ImportError as exc:
            raise RuntimeError(
                "Bench-MR metrics bindings not found. Build them with "
                "./scripts/setup.sh, which produces %s."
                % os.path.join(_BINDINGS_DIR, "metrics_bindings*.so")) from exc
        _mb = metrics_bindings
    return _mb


@contextlib.contextmanager
def quiet():
    """Silence the OMPL console while Bench-MR code runs.

    The metric implementations live inside Bench-MR, which links OMPL, and OMPL
    logs to the process stderr from C++ rather than through Python. Measuring a
    few hundred paths therefore fills the output with identical warnings about
    a state validity checker that is irrelevant here, since nothing is being
    planned. The file descriptor is redirected for the duration of the call, so
    Python exceptions still propagate.
    """
    sys.stderr.flush()
    saved = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(saved, 2)
        os.close(devnull)
        os.close(saved)


def _xy(path):
    p = np.asarray(path, dtype=float)
    if p.shape[0] in (3, 4) and p.shape[0] <= p.shape[1]:
        p = p.T
    return [[float(x), float(y)] for x, y in p[:, :2]]


def _xytheta(path):
    p = np.asarray(path, dtype=float)
    if p.shape[0] in (3, 4) and p.shape[0] <= p.shape[1]:
        p = p.T
    return p


def path_length(path):
    """Length of the path, in grid units."""
    return float(bindings().compute_path_length(_xy(path)))


def curvature(path):
    """``(max, normalized)`` curvature.

    Bench-MR reads curvature from circles through triples of points spaced by
    at least a fixed distance, which is how it avoids the spikes that plain
    finite differences produce on paths containing coincident points.

    The maximum is in inverse grid units. Despite its name, the normalised
    curvature is not divided by the length of the path: it sums the curvature
    of each triple times the length of that triple, so it approximates the
    integral of the curvature along the path, in radians, and grows with the
    path.
    """
    mx, norm = bindings().compute_curvature(_xy(path))
    return float(mx), float(norm)


def max_curvature(path):
    return curvature(path)[0]


def normalized_curvature(path):
    return curvature(path)[1]


def clearing_distances(path, walls, resolution=1.0):
    """Distance to the nearest obstacle at each point of the path, in grid units.

    Measured at the points of the path, that is at the reference point of the
    vehicle, the centre of its rear axle, not from the footprint.

    The Bench-MR binding also writes the map and the distances along the
    path to ``debug.json`` in the working directory
    (``external/NOTICE.md``).
    """
    grid = np.ascontiguousarray(np.asarray(walls, dtype=bool).T)
    return np.asarray(bindings().compute_clearing_distances(
        _xy(path), grid, resolution), dtype=float)


def aol(path):
    """Angle over length: total change of direction over the path length.

    The direction is the direction of travel, read from consecutive positions,
    not the heading of the vehicle; the two differ wherever the vehicle crabs.
    In inverse grid units.
    """
    return float(bindings().compute_aol(_xy(path)))


def approach_heading_error(path, goal_theta, window):
    """Approach Heading Error (AHE): gap between the heading and the goal
    heading over the end of the path, in radians.

    With ``p_1 ... p_n`` the points of the path, ``theta_i`` the heading at
    ``p_i``, ``s_i`` its arc length and ``l`` the length of the path::

        A   = { i < n : s_i >= l - window }
        AHE = sum_{i in A} |theta_i - goal_theta| * |p_{i+1} - p_i|
              / sum_{i in A} |p_{i+1} - p_i|

    each gap wrapped into ``[0, pi]``. Weighting by the step lengths makes the
    value independent of how densely the path is sampled, which differs from
    one planner to the next. If the last step alone is longer than
    ``window``, ``A`` is empty and the gap at the last point is returned.

    Steps shorter than 1e-9 are dropped first, keeping the first point of each
    run of repeated points, so that they neither weigh nor shift the window.

    ``window`` is in grid units, like the path. The thesis takes the length of
    the vehicle, ``vehicle.length * zoom``. The path is read in the order it is
    driven, start to goal.

    Bench-MR has no such metric; this one is computed here, from the poses
    only, for its paths and ours alike.
    """
    q = _xytheta(path)[:, :3]
    step = np.linalg.norm(np.diff(q[:, :2], axis=0), axis=1)
    keep = step > 1e-9
    q = np.vstack([q[:1], q[1:][keep]])
    step = step[keep]
    s = np.r_[0, np.cumsum(step)]
    a = np.where(s >= s[-1] - window)[0]
    w = np.r_[step, 0][a]
    if w.sum() <= 0:
        w = np.ones_like(w)
    gap = np.abs((q[a, 2] - goal_theta + np.pi) % (2 * np.pi) - np.pi)
    return float(np.average(gap, weights=w))


def angular_deviation_at_goal(path, goal_theta):
    """Angular deviation of the final poses, Bench-MR's definition.

    Not a distance to a goal heading: it sums the gap between the last heading
    and the goal heading with the two preceding heading changes, then divides by
    three times the mean spacing between path points.

    Bench-MR also ships a one-argument form that reads the goal from a global
    only a full benchmark run initialises, and dereferences a null pointer
    outside one. It is not wrapped here; this form gives the same number.
    """
    return float(bindings().compute_angular_deviation_at_goal(
        _xytheta(path)[:, :3].tolist(), float(goal_theta)))


def euclidean_deviation_at_goal(path, goal_xy):
    """Euclidean deviation of the final poses, Bench-MR's definition.

    Same shape as :func:`angular_deviation_at_goal`: the distance from the last
    point to the goal plus the last two step lengths, over three times the mean
    spacing. Same remark about the one-argument form Bench-MR ships.

    The result is a ratio, without unit, and depends on how the last points are
    spaced as much as on the distance to the goal: a path that ends exactly on
    the goal with evenly spaced points scores about 2/3, and one whose last
    points are closer together than the rest scores less.
    """
    return float(bindings().compute_euclidean_deviation_at_goal(
        _xytheta(path)[:, :3].tolist(), float(goal_xy[0]), float(goal_xy[1])))


def swept_area(path, vehicle, zoom, supersample=4, step=1):
    """Floor area of the union of successive footprints, in square metres.

    The rasterisation runs in Bench-MR.

    The heading is offset by pi because :meth:`shfm.vehicle.Vehicle.corners`
    puts the front of the vehicle on negative ``x``. The offset applies to any
    path, from this planner or from Bench-MR, and the union does not depend on
    the order the poses come in.
    """
    p = _xytheta(path)
    poses = [[float(x), float(y), float(th + np.pi)] for x, y, th in p[:, :3]]
    footprint = [[float(cx), float(cy)] for cx, cy in vehicle.corners(zoom)]
    return float(bindings().compute_swept_area(
        poses, footprint, vehicle.metres_per_unit(zoom), supersample, step))


def summarize(path, env, goal):
    """All metrics for one path, as a dict.

    The Approach Heading Error is taken over the length of ``env.vehicle``,
    in grid units at ``env.zoom``. It and the deviations look at the end of the
    path, so ``path`` is expected in the order it is driven, and they need the
    goal passed explicitly; see :func:`angular_deviation_at_goal`.
    Through :func:`clearing_distances`, it writes ``debug.json`` in the
    working directory.
    """
    with quiet():
        mx, norm = curvature(path)
        d = clearing_distances(path, env.walls)
        out = {
            "length": path_length(path),
            "max_curvature": mx,
            "normalized_curvature": norm,
            "aol": aol(path),
            "min_clearing": float(d.min()) if d.size else float("nan"),
            "mean_clearing": float(d.mean()) if d.size else float("nan"),
            "swept_area_m2": swept_area(path, env.vehicle, env.zoom),
            "approach_heading_error": approach_heading_error(
                path, goal[2], env.vehicle.length * env.zoom),
        }
        out["angular_deviation"] = angular_deviation_at_goal(path, goal[2])
        out["euclidean_deviation"] = euclidean_deviation_at_goal(path, goal[:2])
    return out
