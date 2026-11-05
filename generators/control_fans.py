#!/usr/bin/env python3
"""Control fans of the four modes, seen from above.

For each mode, the path of the rear axle centre O_R over one unit of time
under each control of :func:`shfm.kinematics.control_set`, at the reference
coefficients (``nu_fwd = 3``, ``nu_2ws = 1.5``, ``nu_4ws = 0.5``,
``gamma_rev = 0.5``), ``delta_F* = delta_R* = 0.5`` rad and the wheelbase of
the default vehicle. The length of a path is the coefficient of its control,
its curvature that of the motion. Paths are integrated by explicit Euler steps
from the pose (0, 0, 0) and drawn at 1.4 units per metre, three times larger
for the manoeuvring modes. The null vector that pads the navigation modes is
left out.

Writes control_fans.tex, the coordinates as TikZ ``plot[smooth] coordinates``
lists, one per control and in drawing order, and control_fans.pdf, the four
fans drawn with matplotlib.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)
OUT = os.environ.get("SHFM_FIGURE_DIR", os.path.join(_REPO, "figures"))
os.makedirs(OUT, exist_ok=True)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from shfm import DEFAULT_VEHICLE
from shfm.kinematics import control_set
from shfm.states import MAN_BWD, MAN_FWD, NAV_BWD, NAV_FWD, SOLVER_ORDER

N_STEPS = 16          # Euler steps over one unit of time
SCALE = 1.4           # drawing units per metre
MANOEUVRE_ZOOM = 3.0  # extra enlargement of the manoeuvring modes

# Drawing order of the controls of each mode, as indices into the control set:
# navigation draws the straight line, then the left and the right turn;
# manoeuvring draws the two crab motions, then the right and the left tight
# turn.
FANS = [
    (NAV_FWD, (2, 0, 1), "état 0 : navigation avant", 1.0),
    (NAV_BWD, (2, 0, 1), "état 1 : navigation arrière", 1.0),
    (MAN_FWD, (0, 1, 3, 2), "état 2 : manœuvre avant (agrandi ×3)", MANOEUVRE_ZOOM),
    (MAN_BWD, (0, 1, 3, 2), "état 3 : manœuvre arrière (agrandi ×3)", MANOEUVRE_ZOOM),
]


def controls_at(theta, mode):
    """The four controls of ``mode`` at heading ``theta``, shape (4, 3)."""
    u = control_set(np.array([theta]), DEFAULT_VEHICLE.wheelbase)
    return u[:, :, 0, SOLVER_ORDER.index(mode)]


def path(mode, index, n_steps=N_STEPS):
    """Positions of O_R under one control over one unit of time, (n + 1, 2)."""
    q = np.zeros(3)
    points = [q[:2].copy()]
    for _ in range(n_steps):
        q = q + controls_at(q[2], mode)[index] / n_steps
        points.append(q[:2].copy())
    return np.array(points)


def fmt(value):
    text = "%.3f" % value
    return "0.000" if text == "-0.000" else text


def tikz_coordinates(points):
    return " ".join("(%s,%s)" % (fmt(x), fmt(y)) for x, y in points)


if __name__ == "__main__":
    lines = []
    fig, axes = plt.subplots(2, 2, figsize=(9, 6))
    for ax, (mode, order, title, zoom) in zip(axes.ravel(), FANS):
        lines.append("%% %s" % mode)
        for index in order:
            pts = SCALE * zoom * path(mode, index)
            lines.append("plot[smooth] coordinates {%s}" % tikz_coordinates(pts))
            ax.plot(pts[:, 0], pts[:, 1], lw=2,
                    color="tab:blue" if mode in (NAV_FWD, NAV_BWD) else "tab:orange")
        ax.add_patch(plt.Rectangle((-0.35, -0.17), 0.7, 0.34, fill=True,
                                   facecolor="0.9", edgecolor="0.3"))
        ax.plot([0], [0], "k.", ms=4)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=10)
        ax.set_xlim(-4.5, 4.5)
        ax.set_ylim(-1.5, 1.5)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "control_fans.pdf"))
    plt.close(fig)
    with open(os.path.join(OUT, "control_fans.tex"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(os.path.join(OUT, "control_fans.tex"))
