"""The four operational modes and the cost of switching between them.

The planner splits the motion of the vehicle into four modes, each with its
own controls, and assigns a cost to every change of mode. This keeps crab
motion out of open corridors, where it serves no purpose, and makes the
returned paths readable by a human operator.

.. warning::
   Two index conventions coexist in this project. The solver stacks the
   modes in the order produced by :mod:`shfm.kinematics`, that is
   ``NAV_FWD, MAN_FWD, MAN_BWD, NAV_BWD``. The thesis numbers them
   ``0`` forward navigation, ``1`` backward navigation, ``2`` forward
   manoeuvring, ``3`` backward manoeuvring. Both describe the same graph with
   the same costs; only the labels differ. Always build the transition matrix
   with :func:`transition_matrix`, which works from names, rather than writing
   the array by hand.
"""
import numpy as np

__all__ = ["NAV_FWD", "MAN_FWD", "MAN_BWD", "NAV_BWD", "SOLVER_ORDER",
           "THESIS_ORDER", "transition_matrix", "to_thesis_index"]

#: Mode names, in the order the solver expects them.
NAV_FWD, MAN_FWD, MAN_BWD, NAV_BWD = "nav_fwd", "man_fwd", "man_bwd", "nav_bwd"

SOLVER_ORDER = (NAV_FWD, MAN_FWD, MAN_BWD, NAV_BWD)
"""Order of the last axis of the control array returned by :mod:`shfm.kinematics`."""

THESIS_ORDER = (NAV_FWD, NAV_BWD, MAN_FWD, MAN_BWD)
"""The order the thesis numbers the states in, 0 to 3."""


def to_thesis_index(mode):
    """Index of ``mode`` in :data:`THESIS_ORDER`."""
    return THESIS_ORDER.index(mode)


def transition_matrix(k1, k2, order=SOLVER_ORDER):
    """Symmetric matrix of mode-switching costs.

    Parameters
    ----------
    k1 : float
        Cost of reversing the direction of travel while staying in navigation.
    k2 : float
        Cost of entering or leaving the manoeuvring mode.
    order : sequence of str
        Mode names in the order the matrix indices should follow. Defaults to
        the order the solver expects.

    Notes
    -----
    Reversing direction *within* the manoeuvring mode costs one unit, the
    smallest entry of the matrix, which is why the admissible range is
    ``1 <= k1 < k2``: switching travel direction must be cheaper than engaging
    the manoeuvring mode, and no transition may be cheaper than that reference.

    Each entry is a constant scalar. It represents the discrete operational
    effort of the switch and does not depend on the velocity or the steering
    angle. The solver scales it by the local environment cost, so a transition
    is priced in the same units as motion at that point.
    """
    if not 1 <= k1 < k2:
        raise ValueError("expected 1 <= k1 < k2, got k1=%r k2=%r" % (k1, k2))
    nav = {NAV_FWD, NAV_BWD}
    man = {MAN_FWD, MAN_BWD}
    n = len(order)
    T = np.zeros((n, n))
    for i, a in enumerate(order):
        for j, b in enumerate(order):
            if a == b:
                continue
            if a in nav and b in nav:
                T[i, j] = k1          # change of travel direction
            elif a in man and b in man:
                T[i, j] = 1.0         # reference, cheapest transition
            else:
                T[i, j] = k2          # navigation <-> manoeuvring
    return T
