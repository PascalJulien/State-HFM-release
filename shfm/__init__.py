"""State-embedded Hamiltonian Fast Marching for four-wheel-steering forklifts.

A path planner that computes globally optimal paths for a vehicle with two
independently steered axles, in the extended configuration space
``(x, y, theta, s)`` where ``s`` is the operational mode.

Typical use::

    from shfm import from_image, Planner, REFERENCE_COEFFICIENTS

    env = from_image("data/maps/two_rooms.png", (100, 100), zoom=4)
    planner = Planner(env, k1=2, k2=3, **REFERENCE_COEFFICIENTS)
    result = planner.solve(seed=[30, 70, 0.0, 0], tips=[[90, 80, 3.14, 0]],
                           free_start_mode=True)
    path = result.path(0)       # (n, 4): x, y, heading, mode, start to goal

``examples/hello_world.py`` does this and draws the path.

The solver needs a CUDA GPU. Everything else, building environments, computing
metrics and drawing, runs on the CPU.
"""
from .vehicle import Vehicle, DEFAULT_VEHICLE
from .states import (NAV_FWD, NAV_BWD, MAN_FWD, MAN_BWD, SOLVER_ORDER,
                     THESIS_ORDER, transition_matrix, to_thesis_index)
from .kinematics import (control_set, control_set_2ws,
                         control_set_unstructured, REFERENCE_COEFFICIENTS)
from .environments import (Environment, from_occupancy, from_image,
                           from_benchmark_run, lift_to_configuration_space)

__version__ = "1.0.0"

#: The part of the package that runs without a GPU.
#:
#: ``Planner``, ``PlanResult`` and ``require_gpu`` are not in this list.
#: They are served by ``__getattr__`` below, so ``from shfm import Planner``
#: works and the solver module is loaded only when it is named. ``cupy`` and
#: ``agd`` are loaded later still, when a solve starts (``require_gpu``).
__all__ = [
    "Vehicle", "DEFAULT_VEHICLE",
    "NAV_FWD", "NAV_BWD", "MAN_FWD", "MAN_BWD", "SOLVER_ORDER",
    "THESIS_ORDER", "transition_matrix", "to_thesis_index",
    "control_set", "control_set_2ws", "control_set_unstructured",
    "REFERENCE_COEFFICIENTS",
    "Environment", "from_occupancy", "from_image", "from_benchmark_run",
    "lift_to_configuration_space",
]

_LAZY = ("Planner", "PlanResult", "require_gpu")


def __getattr__(name):
    # The solver is loaded when one of its names is asked for.
    if name in _LAZY:
        from . import solver
        return getattr(solver, name)
    raise AttributeError(name)


def __dir__():
    # So that tab completion and dir() still show the solver names, even though
    # they are absent from __all__.
    return sorted(list(globals()) + list(_LAZY))
