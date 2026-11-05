#!/usr/bin/env python3
"""The navigation function of the Two-Rooms scene, without the state system.

An isotropic fast marching over ``(x, y, theta)``: unit speed, no obstacle
cost, no state axis. The figure shows the value function of the bare scheme,
rooted at ``ROOT``.

Lengths are in metres (``x`` and ``y`` scaled by ``METRES_PER_UNIT``), and the
orientation stays in radians.

Needs a CUDA device.
"""

# Paths resolved from this file, so that the folder can be moved.
import os as _os
_HERE = _os.path.dirname(_os.path.abspath(__file__))
_REPO = _os.path.dirname(_HERE)
_OUTDIR = _os.environ.get("SHFM_FIGURE_DIR", _os.path.join(_REPO, "figures"))
_os.makedirs(_OUTDIR, exist_ok=True)
import sys as _sys
for _p in (_HERE, _REPO):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import matplotlib as mpl
mpl.rcParams.update({
    'font.size': 10,
    'axes.titlesize': 18,
    'axes.labelsize': 10,
    'savefig.bbox': None,
    'savefig.dpi': 100,
    'mathtext.fontset': 'cm',
    'pdf.fonttype': 42,
})
import matplotlib.pyplot as plt
import numpy as np

from traj_render import render_navfunc

from shfm import DEFAULT_VEHICLE
from shfm.environments import from_occupancy
from shfm.solver import require_gpu

SCENE = _os.path.join(_os.path.abspath(_os.environ.get("SHFM_DATA_DIR")
                                      or _os.path.join(_REPO, "data")),
                      "sensitivity", "env_two_rooms.npz")
ROOT = (30.0, 70.0, np.pi / 2)
N_THETA = 100
METRES_PER_UNIT = 0.25
"""Length of one grid cell, in metres.

The scene is stored on a hundred-square grid whose vehicle scale is 4: its
cells measure 0.25 m and the scene 25 m a side, the scale of the map drawn by
``two_rooms_map.py``.
"""


def navigation_function():
    """Isotropic value function of the scene, rooted at ``ROOT``."""
    xp, Eikonal = require_gpu()
    scene = np.load(SCENE, allow_pickle=True)
    walls = np.asarray(scene["walls"])
    env = from_occupancy(walls, zoom=float(scene["zoom"]), n_theta=N_THETA,
                         vehicle=DEFAULT_VEHICLE)
    walls3 = np.asarray(env.walls3)

    nx, ny = walls.shape
    dx = METRES_PER_UNIT
    hfm = Eikonal.dictIn({"model": "Isotropic3", "verbosity": 0,
                          "seed": [ROOT[0] * dx, ROOT[1] * dx, ROOT[2]],
                          "exportValues": 1})
    hfm.SetRect(sides=[[0, (nx - 1) * dx], [0, (ny - 1) * dx], [0, 2 * np.pi]],
                dims=walls3.shape)
    hfm["periodic"] = (False, False, True)
    hfm["speed"] = xp.asarray(np.ones(walls3.shape))
    hfm["walls"] = xp.asarray(walls3)
    values = hfm.Run()["values"]
    values = values.get() if hasattr(values, "get") else np.asarray(values)
    return dict(U=values, walls_3d=walls3,
                X_x=np.arange(nx) * dx, X_y=np.arange(ny) * dx,
                X_theta=np.linspace(0, 2 * np.pi, N_THETA, endpoint=False),
                meta={"root_point": list(ROOT)})


if __name__ == '__main__':
    scene = navigation_function()
    free = ~scene["walls_3d"]
    finite = np.isfinite(scene["U"][free])
    print("U max %.3f over %d reached voxels (%d out of reach)"
          % (scene["U"][free][finite].max(), finite.sum(), (~finite).sum()))
    render_navfunc(scene)
    plt.savefig(_os.path.join(_OUTDIR, '4WS_nav_function.png'))
    plt.close('all')
    print('4WS_nav_function.png')
