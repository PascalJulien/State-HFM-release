"""Exact start for an extracted path.

The backtracking runs from the goal, which it leaves exactly, to the source,
which it stops near: within the interpolation cube of the source node, up to a
cell and a half and a few degrees away on the campaigns. agd's ``seedRadius``,
which spreads a source over its neighbours with exact local values, needs a
local metric; the bounded-curvature modes have none (a lateral offset of a
fraction of a cell is not a short straight move, it takes an S-turn or a
manoeuvre), so the spreading is not available for this model.

:func:`close_start` corrects the start after the solve instead: it replaces the
first stretch of the path, inside its first mode, by a connection that starts
exactly on the requested pose and joins the path further on.

* Navigation modes: the shortest Dubins path at the minimal turning radius of
  navigation, driven forward or backward as the mode is.
* Manoeuvring modes: a straight move in configuration space, kept only if it is
  a positive combination of the four manoeuvring controls at the start heading
  (crab and counter-phase), the local cone of the mode.

The join is chosen among the path points, between a short minimum distance and
four turning radii, that the connection reaches without being longer than the
stretch it replaces (up to a small tolerance), without straying from that
stretch by more than twice the initial gap (or a cell and a half), and
without entering an obstacle of the lifted map: the one whose connection adds
the least turning to the path. When no point qualifies, the path is returned
unchanged.
"""
import numpy as np

TWO_PI = 2 * np.pi


def _mod(a):
    return np.mod(a, TWO_PI)


def _dubins_words(alpha, beta, d):
    """Candidate (t, p, q, word) for the six Dubins words, normalised radius."""
    sa, ca, sb, cb = np.sin(alpha), np.cos(alpha), np.sin(beta), np.cos(beta)
    cab = np.cos(alpha - beta)
    out = []
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)                      # LSL
    if p2 >= 0:
        tmp = np.arctan2(cb - ca, d + sa - sb)
        out.append((_mod(-alpha + tmp), np.sqrt(p2), _mod(beta - tmp), "LSL"))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)                      # RSR
    if p2 >= 0:
        tmp = np.arctan2(ca - cb, d - sa + sb)
        out.append((_mod(alpha - tmp), np.sqrt(p2), _mod(-beta + tmp), "RSR"))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)                     # LSR
    if p2 >= 0:
        p = np.sqrt(p2)
        tmp = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
        out.append((_mod(-alpha + tmp), p, _mod(-_mod(beta) + tmp), "LSR"))
    p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb)                      # RSL
    if p2 >= 0:
        p = np.sqrt(p2)
        tmp = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
        out.append((_mod(alpha - tmp), p, _mod(beta - tmp), "RSL"))
    tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8                # RLR
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - np.arccos(tmp))
        t = _mod(alpha - np.arctan2(ca - cb, d - sa + sb) + p / 2)
        out.append((t, p, _mod(alpha - beta - t + p), "RLR"))
    tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8               # LRL
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - np.arccos(tmp))
        t = _mod(-alpha - np.arctan2(ca - cb, d + sa - sb) + p / 2)
        out.append((t, p, _mod(_mod(beta) - alpha - t + p), "LRL"))
    return out


def _integrate(q0, segments, radius, step):
    """Sample a forward path made of (kind, length) segments, kind in L, S, R."""
    x, y, th = (float(v) for v in q0)
    out = [np.array([[x, y, th]])]
    for kind, length in segments:
        n = max(1, int(np.ceil(length / step)))
        s = np.linspace(0, length, n + 1)[1:]
        if kind == "S":
            px, py, pt = x + s * np.cos(th), y + s * np.sin(th), np.full_like(s, th)
        else:
            k = 1.0 / radius if kind == "L" else -1.0 / radius
            pt = th + k * s
            px = x + (np.sin(pt) - np.sin(th)) / k
            py = y - (np.cos(pt) - np.cos(th)) / k
        out.append(np.c_[px, py, pt])
        x, y, th = px[-1], py[-1], pt[-1]
    return np.vstack(out)


def dubins_path(q0, q1, radius, step=0.25):
    """Shortest forward Dubins path from ``q0`` to ``q1``, sampled every ``step``.

    Returns ``(points, length)``. The words are tried from the shortest, each
    integrated and kept only if it lands on ``q1``.
    """
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = np.hypot(dx, dy) / radius
    phi = np.arctan2(dy, dx)
    alpha, beta = _mod(q0[2] - phi), _mod(q1[2] - phi)
    words = sorted(_dubins_words(alpha, beta, d), key=lambda w: w[0] + w[1] + w[2])
    for t, p, q, word in words:
        segs = [(word[0], t * radius), (word[1], p * radius), (word[2], q * radius)]
        pts = _integrate(q0, segs, radius, step)
        end = pts[-1]
        dth = np.angle(np.exp(1j * (end[2] - q1[2])))
        if np.hypot(end[0] - q1[0], end[1] - q1[1]) < 1e-3 * radius and abs(dth) < 1e-3:
            return pts, (t + p + q) * radius
    return None, np.inf


def _collides(points, env):
    """True if a pose of ``points`` falls in an obstacle of the lifted map."""
    walls3 = np.asarray(env.walls3)
    nx, ny, nt = walls3.shape
    dx, dy = env.dxy
    i = np.floor(points[:, 0] / dx).astype(int)
    j = np.floor(points[:, 1] / dy).astype(int)
    k = np.round(np.mod(points[:, 2], TWO_PI) / (TWO_PI / nt)).astype(int) % nt
    if (i < 0).any() or (j < 0).any() or (i >= nx).any() or (j >= ny).any():
        return True
    return bool(walls3[i, j, k].any())


def _manoeuvre_cone_ok(delta, theta0, wheelbase_u, delta_f_max, delta_r_max):
    """Is ``delta`` a positive combination of the four manoeuvring controls?"""
    from scipy.optimize import linprog
    from shfm.kinematics import _manoeuvre_block
    U = np.asarray(_manoeuvre_block(np.array([theta0]), wheelbase_u, 1.0,
                                    delta_f_max, delta_r_max))[:, :, 0]   # (4, 3)
    res = linprog(np.ones(4), A_eq=U.T, b_eq=delta, bounds=[(0, None)] * 4,
                  method="highs")
    return res.status == 0


def close_start(path, seed, env, wheelbase_u, delta_f_max, delta_r_max=0.0,
                min_join=1.0, max_join=None, max_skip=2.0, tolerance=0.02,
                step=0.25):
    """Replace the start of ``path`` by an exact connection from ``seed``.

    Parameters
    ----------
    path : ndarray ``(n, 4)``
        As driven, start to goal: ``x, y, theta, mode`` (modes in
        :data:`shfm.states.SOLVER_ORDER`).
    seed : sequence
        Requested start pose ``(x, y, theta)``.
    wheelbase_u : float
        Wheelbase in grid units (``wheelbase * zoom``).
    delta_f_max, delta_r_max : float
        Steering saturations, as given to the control set.
    min_join, max_join : float
        Range of arc length, in grid units, where the join is looked for. The
        default upper bound is four turning radii.
    max_skip : float
        Arc length, in grid units, of mode flicker at the start of the path
        that the connection may replace.
    tolerance : float
        Relative excess length allowed for the connection over what it
        replaces (the stretch of path plus the gap left by the extraction), plus
        half a cell.

    Returns
    -------
    (path, info) : the new path and a dict (``joined`` index or None, ``mode``,
    ``replaced_length``, ``connection_length``, ``gap`` before closing).
    """
    path = np.asarray(path, dtype=float)
    seed = np.asarray(seed[:3], dtype=float)
    radius = wheelbase_u / np.tan(delta_f_max)
    if max_join is None:
        max_join = 4 * radius
    modes = np.round(path[:, 3]).astype(int)
    cuts = np.flatnonzero(np.diff(modes)) + 1
    runs = list(zip(np.r_[0, cuts], np.r_[cuts, len(path)]))
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    info = dict(joined=None, mode=int(modes[0]),
                gap=float(np.hypot(*(path[0, :2] - seed[:2]))),
                gap_theta=float(abs(np.angle(np.exp(1j * (path[0, 2] - seed[2]))))))
    # The extraction often flickers between modes over the last cell before
    # the source, where every mode is seeded at zero: the join may be looked
    # for in any run that starts within ``max_skip`` of the path start, the
    # connection then replacing the flicker.
    from scipy.spatial import cKDTree
    tree = cKDTree(path[:, :2])

    def turning(points):
        return float(np.abs(np.diff(np.unwrap(points[:, 2]))).sum()) if len(points) > 1 else 0.0

    for begin, end in runs:
        if arc[begin] > max_skip:
            break
        mode = int(modes[begin])
        candidates = []
        for m in range(max(begin, 1), end):
            if arc[m] < min_join:
                continue
            if arc[m] > max_join:
                break
            target = path[m, :3]
            if mode in (0, 3):                   # navigation, forward or backward
                if mode == 0:
                    pts, length = dubins_path(seed, target, radius, step)
                else:
                    flip = np.array([0, 0, np.pi])
                    pts, length = dubins_path(seed + flip, target + flip, radius, step)
                    if pts is not None:
                        pts = pts - flip
            else:                                # manoeuvre: straight in (x, y, theta)
                delta = target - seed
                delta[2] = np.angle(np.exp(1j * delta[2]))
                delta_fwd = -delta if mode == 2 else delta
                if not _manoeuvre_cone_ok(delta_fwd, seed[2], wheelbase_u,
                                          delta_f_max, delta_r_max):
                    continue
                length = float(np.hypot(delta[0], delta[1]))
                n = max(2, int(np.ceil(length / step)) + 1)
                pts = seed + np.linspace(0, 1, n)[:, None] * delta
            if pts is None:
                continue
            # The connection also covers the gap between the requested pose
            # and the first point of the path, left uncovered by the extraction.
            replaced = arc[m] + info["gap"]
            if length > replaced * (1 + tolerance) + 0.5:
                continue
            if _collides(pts, env):
                continue
            # The connection must stay near the stretch it replaces, which the
            # solver shaped with the obstacle cost.
            dev = tree.query(pts[:, :2])[0].max()
            if dev > max(2.0 * info["gap"], 1.5):
                continue
            # Turning the connection adds to what the path itself does over
            # the stretch it replaces: the first admissible join is often a
            # sharp S-turn, a later one a gentle drift.
            excess = turning(pts) - turning(path[:m + 1])
            candidates.append((round(excess, 2), m, pts, length, replaced))
        if candidates:
            _, m, pts, length, replaced = min(candidates, key=lambda c: (c[0], c[1]))
            conn = np.c_[pts[:-1], np.full(len(pts) - 1, float(mode))]
            out = np.vstack([conn, path[m:]])
            info.update(joined=int(m), mode=mode, replaced_length=float(replaced),
                        connection_length=float(length))
            return out, info
    return path, info
