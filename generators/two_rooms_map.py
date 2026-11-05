#!/usr/bin/env python3
"""Map of the Two-Rooms scene, drawn beside its navigation function.

The scene used by ``nav_function.py``, with the source ``ROOT`` (position and
orientation). Lengths are in metres at the cell size of the stored scene,
``1 / zoom`` (0.25 m): the scene measures 25 m a side, as in section 5.2.4 of
the thesis, and the scale of the value function of ``nav_function.py``.

Same frame as the 3D plot of ``render_navfunc``: the grid is indexed
``walls[i, j]`` with ``x`` along axis 0 and ``y`` along axis 1 (the order of the
seed and of ``SetRect`` in ``nav_function.py``), and node ``(i, j)`` sits at
``(i * dx, j * dx)``. No transposition: ``x`` to the right, ``y`` up, as the
array seen from above in the 3D plot.

No GPU computation: only the occupancy of the scene is read.
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
    'axes.labelsize': 10,
    'savefig.bbox': None,
    'savefig.dpi': 100,
    'mathtext.fontset': 'cm',
    'pdf.fonttype': 42,
})
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from matplotlib.ticker import FuncFormatter, MultipleLocator

from nav_function import ROOT, SCENE

OBSTACLE_COLOR = '#404040'
SOURCE_COLOR = '#0050d0'
ARROW_LENGTH = 2.5
"""Length of the heading arrow, in metres (ten cells)."""


def walls_path(walls, dx):
    """All obstacles as a single path: cell (i, j) covers (i +- 1/2) dx.

    Cells are merged into runs along x. Filling everything at once avoids thin
    seams between neighbouring cells on screen.
    """
    nx, ny = walls.shape
    verts, codes = [], []
    for j in range(ny):
        i = 0
        while i < nx:
            if not walls[i, j]:
                i += 1
                continue
            k = i
            while k < nx and walls[k, j]:
                k += 1
            x0, x1 = (i - 0.5) * dx, (k - 0.5) * dx
            y0, y1 = (j - 0.5) * dx, (j + 0.5) * dx
            verts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
            codes += [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO,
                      Path.CLOSEPOLY]
            i = k
    return Path(verts, codes)


def decimal_comma(value, _pos):
    return ('%g' % value).replace('.', ',')


def render_map(ax):
    scene = np.load(SCENE, allow_pickle=True)
    walls = np.asarray(scene["walls"], dtype=bool)
    nx, ny = walls.shape
    dx = 1.0 / float(scene["zoom"])

    ax.add_patch(PathPatch(walls_path(walls, dx), facecolor=OBSTACLE_COLOR,
                           edgecolor='none', linewidth=0))
    ax.set_xlim(-0.5 * dx, (nx - 0.5) * dx)
    ax.set_ylim(-0.5 * dx, (ny - 0.5) * dx)
    ax.set_aspect('equal')
    ax.set_facecolor('white')

    x0, y0, theta0 = ROOT[0] * dx, ROOT[1] * dx, ROOT[2]
    ax.annotate('', xy=(x0 + ARROW_LENGTH * np.cos(theta0),
                        y0 + ARROW_LENGTH * np.sin(theta0)),
                xytext=(x0, y0),
                arrowprops=dict(arrowstyle='-|>', color=SOURCE_COLOR, lw=1.6,
                                mutation_scale=11, shrinkA=0, shrinkB=0),
                zorder=4)
    ax.plot([x0], [y0], 'o', color=SOURCE_COLOR, markersize=5.5, zorder=5)

    ax.xaxis.set_major_locator(MultipleLocator(4))
    ax.yaxis.set_major_locator(MultipleLocator(4))
    ax.xaxis.set_major_formatter(FuncFormatter(decimal_comma))
    ax.yaxis.set_major_formatter(FuncFormatter(decimal_comma))
    ax.set_xlabel(r'$x$ (m)')
    ax.set_ylabel(r'$y$ (m)')
    for spine in ax.spines.values():
        spine.set_linewidth(0.8)
    return ax


if __name__ == '__main__':
    fig, ax = plt.subplots(figsize=(3.2, 3.2))
    render_map(ax)
    out = _os.path.join(_OUTDIR, '4WS_nav_function_map.pdf')
    fig.savefig(out, bbox_inches='tight', pad_inches=0.02)
    plt.close(fig)
    print(out)
