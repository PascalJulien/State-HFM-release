"""Running the state-embedded Hamiltonian Fast Marching solver.

This is a thin wrapper around the ``DubinsState4`` model of the
`agd <https://github.com/Mirebeau/AdaptiveGridDiscretizations>`_ library by
J.-M. Mirebeau, which provides the anisotropic eikonal solver and its CUDA
kernels. This module adds the inputs specific to the forklift: the four-mode
control set, the transition matrix and the obstacle cost.

A GPU with CUDA and ``cupy`` is required to solve. They are loaded when a
solve starts, by :func:`require_gpu`, which raises with an explicit message
when they are missing.
"""
import os
import sys
import warnings

import numpy as np

from .kinematics import control_set
from .states import transition_matrix

__all__ = ["Planner", "PlanResult", "require_gpu", "agd_location"]

_VENDORED_AGD = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "external", "agd")


def _prefer_vendored_agd():
    """Put the copy of agd shipped with this repository first on the path.

    The library is vendored under ``external/agd`` so that the repository does
    not depend on a package index remaining available. When the directory is
    present it takes precedence over any installed copy, which keeps results
    reproducible across machines.
    """
    if os.path.isdir(os.path.join(_VENDORED_AGD, "agd")):
        if _VENDORED_AGD not in sys.path:
            sys.path.insert(0, _VENDORED_AGD)


def agd_location():
    """Path of the agd package in use."""
    _prefer_vendored_agd()
    import agd
    return os.path.dirname(agd.__file__)


def require_gpu():
    """Import the GPU stack, with an explicit message when it is missing."""
    try:
        import cupy  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            "shfm.solver needs a CUDA GPU with cupy installed. "
            "See environment.yml for a working set of versions."
        ) from exc
    _prefer_vendored_agd()
    from agd import Eikonal
    from agd import AutomaticDifferentiation as ad
    # cupy_friendly prints on stdout what it rebinds, which is useful once and
    # repeats in every notebook cell that triggers it; it is discarded here.
    import contextlib, io as _io
    with contextlib.redirect_stdout(_io.StringIO()):
        xp = ad.cupy_friendly(np)
        Eikonal = ad.cupy_friendly(Eikonal)
    return xp, Eikonal


class PlanResult:
    """What one solve returns.

    Attributes
    ----------
    values : ndarray ``(nx, ny, n_theta, 4)``
        The navigation function: cost to reach each configuration and mode from
        the start.
    geodesics : list of ndarray
        One path per requested goal, as ``(4, n_points)`` arrays holding
        ``x, y, theta, state``, exactly as the solver returns them: backtracked,
        so they run from the goal to the start. Prefer :meth:`path`, which
        returns them the way they are travelled.
    seed, tips : the start and the goals asked for, as ``(x, y, theta)``.
    env : the :class:`shfm.environments.Environment` of the solve.
    hfm_in, hfm_out : the input and output dictionaries of agd;
        ``hfm_out["geodesic_stopping_criteria"]`` holds the stopping
        criterion of each geodesic.
    """

    def __init__(self, values, geodesics, hfm_in, hfm_out, seed=None,
                 tips=None, env=None):
        self.values = values
        self.geodesics = geodesics
        self.hfm_in = hfm_in
        self.hfm_out = hfm_out
        self.seed = seed
        self.tips = tips
        self.env = env

    def reached(self, index=0):
        """Whether the path to goal ``index`` joins the goal to the start.

        False in three cases. The start is outside the grid: agd does not
        reject such a seed but wraps its index around, so the front starts
        from another cell. The goal could not be reached at all: its path is
        a single point, stopped ``InWall``, because the goal is outside the
        grid, in collision, cut off from the start, or the start itself is
        in collision. Or the backtracking stopped further than one vehicle
        length from the start, the definition of a degenerate path in the
        studies. Within that distance a path counts as reached whatever its
        stopping criterion: valid paths also stop ``Stationnary`` or
        ``PastSeed`` close to the start, and :func:`shfm.splice.close_start`
        closes the gap.

        :meth:`Planner.solve` warns, with the probable cause, for every goal
        for which this is false.
        """
        return _why_not_reached(self, index) is None

    def path(self, index=0):
        """Geodesic ``index`` as ``(n_points, 4)``: ``x, y, theta, state``.

        Ordered the way it is driven, from start to goal. The solver backtracks
        from the goal, so its raw output runs the other way.
        """
        return np.asarray(self.geodesics[index]).T[::-1]

    def raw_path(self, index=0):
        """The geodesic exactly as the solver produced it, goal to start.

        The comparison plates measure the curvature of our paths in this
        order, goal to start, on the path cleaned by
        :func:`shfm.postprocess.dechatter` and :func:`shfm.splice.close_start`,
        and the Approach Heading Error in the order of :meth:`path` (see
        :func:`shfm.plates.custom_stats`). Bench-MR forms its curvature triples
        from one end, so the traversal order changes the normalised curvature
        by a few percent.
        """
        return np.asarray(self.geodesics[index]).T

    def modes_used(self, index=0):
        """Share of the points of geodesic ``index`` in each mode.

        Keyed by the index of the mode in the control set, which for the
        default four-mode set is its position in
        :data:`shfm.states.SOLVER_ORDER`, not the numbering of
        :data:`shfm.states.THESIS_ORDER`.
        """
        s = self.path(index)[:, 3].round().astype(int)
        return {k: float((s == k).mean()) for k in range(self.values.shape[-1])}


class Planner:
    """Plan paths for the forklift on a given environment.

    Parameters
    ----------
    env : shfm.environments.Environment
    k1, k2 : float
        Transition costs, with ``1 <= k1 < k2``.
    obstacle_cost : bool
        When true, the local cost grows near obstacles, which pushes the path
        away from walls. When false, the path is a pure geodesic and passes
        close to them.
    saturate : bool
        Cap the clearance term, at the ``saturation_cap`` the environment was
        built with (the vehicle length by default). Without the cap the path
        drifts towards the centre of open areas, which lengthens it for no gain
        in safety.
    fm2 : float
        Weight of the clearance term in the speed field.
    """

    def __init__(self, env, k1=10.0, k2=50.0, obstacle_cost=True,
                 saturate=True, fm2=1.0, controls=None, transitions=None,
                 **coefficients):
        self.env = env
        self.k1, self.k2 = k1, k2
        # Options for studying variants: a control set with a different
        # number of modes, given as an array or as a callable called as
        # ``f(theta, wheelbase_u, xp)``, and the matching transition matrix.
        # The builders of shfm.kinematics take xp as a keyword, so wrap them:
        # ``lambda th, wb, xp: control_set_2ws(th, wb, xp=xp)``.
        self._controls = controls
        self._transitions = transitions
        self.obstacle_cost = obstacle_cost
        self.saturate = saturate
        self.fm2 = fm2
        self.coefficients = coefficients

    def controls(self, xp=np):
        """Control set for this planner's coefficients."""
        if self._controls is not None:
            # Either a ready-made array, or a callable building one for the
            # array module in use.
            if callable(self._controls):
                wheelbase_u = self.env.vehicle.wheelbase * self.env.zoom
                return self._controls(self.env.theta, wheelbase_u, xp)
            return xp.asarray(self._controls)
        wheelbase_u = self.env.vehicle.wheelbase * self.env.zoom
        return control_set(self.env.theta, wheelbase_u, xp=xp,
                           **self.coefficients)

    def transitions(self, n_states):
        """Transition matrix for a control set holding ``n_states`` states.

        The shape is checked against the control set.

        A zero off-diagonal entry is warned about, not refused. ``agd`` passes
        this matrix to the kernel as ``state_transition_costs ** -2``, so a zero
        cost arrives as an infinite stencil weight and leaves part of
        the value function at infinity: on Two-Rooms, 425 557 voxels more than
        the same problem posed without a transition term. Free switching is
        expressed by folding the controls into a single state, as
        :func:`shfm.kinematics.control_set_unstructured` does, not by a zero
        matrix; with ``n_states == 1`` the kernel skips the transition term and
        this matrix is never read.

        It is a warning rather than an error, so that an exploration outside
        the admissible range can still run.
        """
        if self._transitions is not None:
            T = np.asarray(self._transitions, dtype=float)
            if T.shape != (n_states, n_states):
                raise ValueError(
                    "transition matrix is %s but the control set has %d state(s)"
                    % (T.shape, n_states))
            if n_states > 1 and not np.all(T[~np.eye(n_states, dtype=bool)] > 0):
                import warnings
                warnings.warn(
                    "transition matrix has a zero off-diagonal entry; the "
                    "solver squares and inverts it, so that transition reaches "
                    "the kernel as an infinite stencil weight and part of the "
                    "value function will be left at infinity. For free "
                    "switching, put the controls in a single state "
                    "instead.", RuntimeWarning, stacklevel=2)
            return T
        if n_states == 1:
            # Never read by the kernel; kept finite so that the ``** -2`` the
            # library applies to it does not produce an infinity.
            return np.ones((1, 1))
        T = transition_matrix(self.k1, self.k2)
        if T.shape != (n_states, n_states):
            raise ValueError(
                "the default four-mode transition matrix does not fit a control "
                "set with %d state(s); pass transitions= explicitly" % n_states)
        return T

    def solve(self, seed, tips, verbosity=0, geodesic_mixed_state=True,
              free_start_mode=False, free_goal_mode=False, **kwargs):
        """Propagate the front from ``seed`` and backtrack to each of ``tips``.

        ``seed`` and ``tips`` are ``(x, y, theta, state)`` quadruples; the last
        component picks the mode the vehicle starts or ends in.

        ``geodesic_mixed_state`` lets the backtracking cross mode boundaries.
        It must stay on for a multi-mode model: without it the state is treated
        as an ordinary coordinate and interpolated, so that the mode column of
        the path holds values between the integers, or the path stops at the
        first switch.

        ``free_start_mode`` seeds the start in every mode at zero cost, instead
        of in the mode ``seed`` names: a vehicle at rest may leave in any
        mode without a switching cost. It also lets the backtracking end
        properly. When the start is imposed in a mode the path does not leave
        in, the 2WS backtracking switches mode within a cell of the start and
        then oscillates there, a quarter of a cell at a time, until
        the solver's stall criterion stops it (``Stationnary``, 64 steps by
        default); seeded in every mode, it stops on the seed (``AtSeed``).
        Measured on Two-Rooms read with ``from_image`` at 200 cells, 2WS,
        ``k1=10, k2=50``: 61 of the 64 reversals of the path, and 14.7 units
        of its length, were that stall. With ``k1=2, k2=3``, or on the grid of
        the campaign, the path from an imposed start does not oscillate.

        ``free_goal_mode`` does the same at the goal: each tip is tried in every
        mode and the geodesic of the cheapest one is kept. The chosen modes are
        in ``hfm_out["goal_modes"]``.

        Both are off by default.
        """
        xp, Eikonal = require_gpu()
        env = self.env
        controls = self.controls(xp)
        n_states = controls.shape[-1]

        if free_goal_mode:
            asked = [list(t) for t in tips]
            tips = [t[:3] + [s] for t in asked for s in range(n_states)]
        hfm_in = Eikonal.dictIn({
            "model": "DubinsState4",
            "tips": [list(t) for t in tips],
            "origin": (0, 0, 0, -0.5),
            "gridScales": (env.dxy[0], env.dxy[1],
                           2 * np.pi / env.n_theta, 1),
            "periodic": [False, False, True, False],
            "exportValues": True,
            "exportActiveNeighs": True,
            # The default cap is low enough that the front of a large grid
            # can need more outer iterations; agd then raises ValueError
            # ("did not reach convergence").
            "nitermax_o": 4000,
            "geodesic_mixed_state": geodesic_mixed_state,
            "verbosity": verbosity,
        })
        if free_start_mode:
            hfm_in["seeds"] = np.array([list(seed)[:3] + [s]
                                        for s in range(n_states)], dtype=float)
        else:
            hfm_in["seed"] = list(seed)
        hfm_in["controls"] = controls
        hfm_in["state_transition_costs"] = self.transitions(n_states)
        hfm_in["dims"] = (env.shape[0], env.shape[1], env.n_theta, n_states)

        clearance = (env.walls3_dist_saturated if self.saturate
                     else env.walls3_dist)
        if self.obstacle_cost:
            speed = 1 + self.fm2 * xp.array(clearance)[:, :, :, np.newaxis]
        else:
            speed = np.ones(hfm_in.shape)
        hfm_in["speed"] = np.broadcast_to(speed, hfm_in.shape)
        hfm_in["walls"] = np.broadcast_to(
            xp.array(env.walls3[:, :, :, np.newaxis]), hfm_in.shape)
        hfm_in.update(**kwargs)

        hfm_out = hfm_in.Run()
        to_np = lambda a: a.get() if hasattr(a, "get") else np.asarray(a)
        values = to_np(hfm_out["values"])
        geodesics = [to_np(g) for g in hfm_out["geodesics"]]
        if free_goal_mode:
            def value_at(tip):
                index = np.asarray(to_np(hfm_in.IndexFromPoint(tip)[0]))
                index[2] %= values.shape[2]                  # periodic heading
                return float(values[tuple(index)])
            stops = hfm_out.get("geodesic_stopping_criteria")
            keep, modes, kept_stops = [], [], []
            for i in range(len(asked)):
                cands = list(range(i * n_states, (i + 1) * n_states))
                best = int(np.argmin([value_at(tips[j]) for j in cands]))
                keep.append(geodesics[cands[best]])
                modes.append(best)
                if stops is not None:
                    kept_stops.append(stops[cands[best]])
            geodesics = keep
            hfm_out["goal_modes"] = modes
            if stops is not None:
                hfm_out["geodesic_stopping_criteria"] = kept_stops
        result = PlanResult(values, geodesics, hfm_in, hfm_out,
                            seed=[float(v) for v in list(seed)[:3]],
                            tips=[[float(v) for v in list(t)[:3]]
                                  for t in (asked if free_goal_mode else tips)],
                            env=env)
        _warn_unreached(result)
        return result


def _pose_status(env, pose):
    """``"outside"``, ``"collision"`` or ``"free"``: where the solver reads a pose.

    agd places node ``i`` at ``(i + 1/2) h`` and interpolates a point from
    the nodes of the cell around it. The pose counts as in collision when
    every one of them is.
    """
    walls3 = np.asarray(env.walls3)
    nx, ny, nt = walls3.shape
    hx, hy = env.dxy
    if not (0 <= pose[0] < nx * hx and 0 <= pose[1] < ny * hy):
        return "outside"
    ci, cj = np.floor(pose[0] / hx - 0.5), np.floor(pose[1] / hy - 0.5)
    ck = np.floor(np.mod(pose[2], 2 * np.pi) / (2 * np.pi / nt) - 0.5)
    ii = np.clip([ci, ci + 1], 0, nx - 1).astype(int)
    jj = np.clip([cj, cj + 1], 0, ny - 1).astype(int)
    kk = np.mod([ck, ck + 1], nt).astype(int)
    return "collision" if walls3[np.ix_(ii, jj, kk)].all() else "free"


def _start_problem(result):
    """Why no goal can be reached from the start, or None.

    A start outside the grid comes first: agd wraps its index around and
    propagates from another cell, so values are finite all the same.
    """
    status = _pose_status(result.env, result.seed)
    if status == "outside":
        return "the start, %s, is outside the grid" % _fmt(result.seed)
    if np.isfinite(result.values).any():
        return None
    if status == "collision":
        return "the vehicle is in collision at the start, %s" % _fmt(result.seed)
    return "nothing is reachable from the start, %s" % _fmt(result.seed)


def _fmt(pose):
    return "(%.1f, %.1f, %.2f)" % tuple(pose[:3])


def _why_not_reached(result, index):
    """The probable reason why goal ``index`` was not reached, or None."""
    g = np.asarray(result.geodesics[index])            # (4, n), goal to start
    stops = result.hfm_out.get("geodesic_stopping_criteria")
    stop = stops[index] if stops is not None else None
    single = g.ndim < 2 or g.shape[-1] <= 1
    if single and stop == "AtSeed":                     # the goal is the start
        return None
    env, seed, tips = result.env, result.seed, result.tips
    if env is None or seed is None or tips is None:
        return "the path is a single point" if single else None
    if _pose_status(env, seed) == "outside":
        return _start_problem(result)
    gap = float(np.hypot(g[0, -1] - seed[0], g[1, -1] - seed[1]))
    reach = env.vehicle.length * env.zoom
    if not single and gap <= reach:
        return None
    if single:
        problem = _start_problem(result)
        if problem:
            return problem
        status = _pose_status(env, tips[index])
        if status == "outside":
            return "the goal is outside the grid"
        if status == "collision":
            return "the vehicle is in collision at the goal pose"
        return ("the goal cannot be reached from the start: it is enclosed, or "
                "behind a passage the vehicle does not fit through")
    return ("the path stops %.1f grid units from the start, more than the "
            "vehicle length (stopping criterion %s)" % (gap, stop))


def _warn_unreached(result):
    """One RuntimeWarning per goal that :meth:`PlanResult.reached` rejects.

    A single one when the start is the cause. Every message starts with
    ``goal``, so that ``warnings.filterwarnings("error", message="goal",
    category=RuntimeWarning)`` turns them into errors.
    """
    n = len(result.geodesics)
    failed = [(i, _why_not_reached(result, i)) for i in range(n)]
    failed = [(i, why) for i, why in failed if why]
    if not failed:
        return
    start = _start_problem(result) if result.env is not None else None
    if n > 1 and start and all(why == start for _, why in failed):
        warnings.warn("goals 0 to %d of %d not reached: %s."
                      % (n - 1, n, start), RuntimeWarning, stacklevel=3)
        return
    for i, why in failed:
        pose = _fmt(result.tips[i]) if result.tips is not None else ""
        warnings.warn("goal %d of %d, %s, not reached: %s; "
                      "PlanResult.reached(%d) is False."
                      % (i, n, pose, why, i), RuntimeWarning, stacklevel=3)
