"""Removal of the extraction chatter from a path.

Near the seed, the backtracking of a geodesic can step back and forth along
the heading, a quarter of a cell each way, for tens of points, before it
stops. Each of those steps reverses the direction of travel, so the chatter
reads as dozens of cusps: it adds pi per step to the angle-over-length,
three-point circles of a few hundredths of a cell to the maximum curvature,
and its length to the path length.

:func:`dechatter` collapses every run of three or more reversals spaced by at
most two points, which no manoeuvre of the vehicle produces, to its last
point. Isolated reversals, the real cusps, are kept. The paths of the
comparison studies are cleaned this way, then their start is closed on the
requested pose by :func:`shfm.splice.close_start`.
"""
import numpy as np

__all__ = ["reversals", "chatter_runs", "dechatter"]


def reversals(path):
    """Indices ``i`` (into the steps) where the direction of travel flips.

    The direction is the sign of the scalar product between the displacement
    and the heading vector.
    """
    p = np.asarray(path, dtype=float)
    d = np.diff(p[:, :2], axis=0)
    step = np.linalg.norm(d, axis=1)
    h = np.c_[np.cos(p[:-1, 2]), np.sin(p[:-1, 2])]
    dot = np.sum(d * h, axis=1)
    idx = np.where(step > 1e-6)[0]
    s = np.sign(dot[idx])
    return idx[1:][s[1:] != s[:-1]]


def chatter_runs(path, min_flips=3, max_gap=2):
    """Runs of reversals spaced by at most ``max_gap`` steps, ``min_flips`` long or more."""
    f = reversals(path)
    runs, cur = [], []
    for i in f:
        if cur and i - cur[-1] > max_gap:
            if len(cur) >= min_flips:
                runs.append((cur[0], cur[-1]))
            cur = []
        cur.append(i)
    if len(cur) >= min_flips:
        runs.append((cur[0], cur[-1]))
    return runs


def dechatter(path):
    """Path with every chatter run collapsed to one point.

    Returns ``(path, n_removed, length_removed)``.
    """
    p = np.asarray(path, dtype=float)
    keep = np.ones(len(p), dtype=bool)
    removed_len = 0.0
    for a, b in chatter_runs(p):
        # points a .. b + 1 lie in the chatter; the last one is kept
        lo, hi = max(a - 1, 0), b + 1
        seg = p[lo:hi + 1, :2]
        removed_len += float(np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1))
                             - np.linalg.norm(seg[-1] - seg[0]))
        keep[lo + 1:hi] = False
    return p[keep], int((~keep).sum()), removed_len
