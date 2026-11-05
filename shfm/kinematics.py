"""Admissible velocities of the forklift, per operational mode.

The planner is generative: instead of steering the vehicle, it is given the
finite set of velocity vectors the vehicle may follow at each orientation, and
it propagates a front through them. This module builds that set.

A four-wheel-steering vehicle with both axles at their limits admits four
extreme velocities, obtained from the four sign combinations of the front and
rear steering angles. Two of them are tight turns, the other two are crab
motions, where the chassis translates sideways while keeping its heading.
Locking the rear axle collapses the set to the car-like model.

The velocities are those of the reference point, the centre of the rear axle.
With the front wheels steered by ``delta_F`` and the rear ones by ``delta_R``,
rolling without slip makes that point move along ``theta + delta_R`` and turns
the chassis at

    theta' = v cos(delta_R) (tan(delta_F) - tan(delta_R)) / L

for a speed ``v`` of the rear-axle centre and a wheelbase ``L``. The factor
``cos(delta_R)`` expresses that both axles have the same speed along the
chassis; without it the front wheels would slip sideways whenever the rear axle
steers. It leaves crab motion (``delta_F = delta_R``, no rotation) and car-like
motion (``delta_R = 0``) unchanged, and slows the rotation of counter-phase
turns.

Every vector carries a *speed coefficient*. These coefficients are
dimensionless weights, not physical speeds: the vehicle is assumed to travel at
constant speed, and the quantity the solver minimises is a cost of travel, not
a duration. Raising a coefficient lengthens the corresponding vector, which
lowers the cost of paths that use it and therefore makes that motion
more attractive.
"""
import numpy as np

from .states import SOLVER_ORDER

__all__ = ["control_set", "control_set_2ws", "control_set_unstructured",
           "REFERENCE_COEFFICIENTS"]

REFERENCE_COEFFICIENTS = dict(nu_fwd=3.0, nu_2ws=1.5, nu_4ws=0.5, gamma_rev=0.5)
"""Speed coefficients of the reference configuration.

They satisfy the intended ordering ``nu_fwd > nu_2ws > nu_4ws``: straight
forward navigation is preferred, then 2WS turning, and 4WS manoeuvring only
where the geometry requires it.
"""


def _navigation_block(theta, wheelbase_u, nu_fwd, nu_2ws, delta_f_max, xp=np):
    """Three navigation velocities, padded to four.

    Turning left, turning right, and going straight. The fourth entry is a null
    vector: the solver takes a single ``ncontrols`` for every mode, so the
    navigation modes, which have one control fewer than the manoeuvring modes,
    are padded. A null velocity is never selected, it only fills the slot.
    """
    zero, one = np.zeros_like(theta), np.ones_like(theta)
    return xp.array([
        (nu_2ws * np.cos(theta), nu_2ws * np.sin(theta),
         nu_2ws * np.tan(delta_f_max) / (wheelbase_u * one)),
        (nu_2ws * np.cos(theta), nu_2ws * np.sin(theta),
         nu_2ws * np.tan(-delta_f_max) / (wheelbase_u * one)),
        (nu_fwd * np.cos(theta), nu_fwd * np.sin(theta), zero),
        (zero, zero, zero),
    ])


def _manoeuvre_block(theta, wheelbase_u, nu_4ws, delta_f_max, delta_r_max, xp=np):
    """Four manoeuvring velocities: two crab motions and two tight turns.

    One per sign combination of the steering angles at saturation, in the order
    ``(+F, +R)``, ``(-F, -R)`` (crab) and ``(+F, -R)``, ``(-F, +R)``
    (counter-phase). Each is the velocity of the rear-axle centre: direction
    ``theta + delta_R``, rotation ``cos(delta_R) (tan delta_F - tan delta_R) / L``.

    With the cosine, the counter-phase turn at full lock is the tightest one only
    while ``delta_f_max + delta_r_max <= pi / 2``. Beyond, a smaller rear angle
    turns tighter, and these four vectors describe a vehicle slightly less agile
    than the model; the approximation is conservative.
    """
    one = np.ones_like(theta)
    rows = []
    for df, dr in ((delta_f_max, delta_r_max), (-delta_f_max, -delta_r_max),
                   (delta_f_max, -delta_r_max), (-delta_f_max, delta_r_max)):
        rate = (np.tan(df) - np.tan(dr)) / wheelbase_u
        rate = rate * np.cos(dr)
        rows.append((np.cos(theta + dr), np.sin(theta + dr), rate * one))
    return nu_4ws * xp.array(rows)


def control_set(theta, wheelbase_u, nu_fwd=3.0, nu_2ws=1.5, nu_4ws=0.5,
                gamma_rev=0.5, delta_f_max=0.5, delta_r_max=0.5, xp=np):
    """Velocities of the four modes, for every orientation.

    Parameters
    ----------
    theta : ndarray
        Sampled orientations, shape ``(n_theta,)``.
    wheelbase_u : float
        Wheelbase expressed in grid units, that is ``wheelbase * zoom``. The
        translations are in grid units and the angular rate in radians per grid
        unit, so the two must share the same scale.
    nu_fwd, nu_2ws, nu_4ws : float
        Speed coefficients of straight navigation, of 2WS turning and of
        manoeuvring.
    gamma_rev : float
        Coefficient in ``[0, 1]`` applied to the backward navigation mode. A
        value below one discourages unnecessary reversing, zero forbids it.
        It does not apply to the manoeuvring modes: reversing there leaves the
        norm of the control unchanged.
    delta_f_max, delta_r_max : float
        Steering saturations of the front and rear axles, in radians.

    Returns
    -------
    ndarray
        Shape ``(4, 3, n_theta, 4)``: four controls, three configuration-space
        components, the orientations, and the four modes in the order given by
        :data:`shfm.states.SOLVER_ORDER`.
    """
    nav_fwd = _navigation_block(theta, wheelbase_u, nu_fwd, nu_2ws,
                                delta_f_max, xp)
    man_fwd = _manoeuvre_block(theta, wheelbase_u, nu_4ws,
                               delta_f_max, delta_r_max, xp)
    blocks = {
        "nav_fwd": nav_fwd,
        "man_fwd": man_fwd,
        "man_bwd": -man_fwd,
        "nav_bwd": -gamma_rev * nav_fwd,
    }
    return np.stack([blocks[name] for name in SOLVER_ORDER], axis=-1)


def control_set_2ws(theta, wheelbase_u, **kwargs):
    """Same four-mode control set, restricted to front-wheel steering.

    Setting the rear saturation to zero collapses the manoeuvring modes onto
    car-like motion. This is the baseline to compare against: the same
    planner with the rear axle locked.
    """
    kwargs["delta_r_max"] = 0.0
    return control_set(theta, wheelbase_u, **kwargs)


def control_set_unstructured(theta, wheelbase_u, delta_f_max=1.05,
                             delta_r_max=1.05, xp=np):
    """The four-wheel-steering controls alone, with no mode structure.

    This is the baseline the state system is meant to improve on: the planner
    receives the full 4WS kinematics, forward and backward, and nothing else.
    Nothing then discourages crab motion, and it appears everywhere, including
    along open corridors where it serves no purpose.

    Returns an array of shape ``(8, 3, n_theta, 1)``: a single state
    carrying the eight extreme steering vectors, the four of the manoeuvring
    block and their four reverses.

    .. note::
       The eight controls belong to one state. Splitting them into
       two free-switching states instead, and passing a zero transition matrix,
       looks equivalent and is not: ``agd`` squares and inverts that matrix
       (``state_transition_costs ** -2`` in ``Eikonal/HFM_CUDA/_Kernel.py``), so
       a zero entry reaches the CUDA kernel as an infinite stencil weight, and
       only the diagonal is skipped there. Measured on Two-Rooms, the value
       function then comes back with 425 557 extra voxels left at infinity, with
       no error raised and a plausible-looking path returned. With a single
       state the kernel skips the transition term altogether.
    """
    man = _manoeuvre_block(theta, wheelbase_u, 1.0, delta_f_max, delta_r_max, xp)
    merged = np.concatenate([man, -man], axis=0)      # (8, 3, n_theta)
    return merged[..., np.newaxis]
