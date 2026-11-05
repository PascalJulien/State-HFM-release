#!/usr/bin/env python3
"""Disc covering of the vehicle footprint, at 4, 8 and 16 discs.

The rectangle of the forklift is covered by ``n`` equal discs whose centres are
spread evenly along its length; the radius is the smallest one that covers the
rectangle. Each panel also marks the reference point O_R, the centre of the
rear axle, and the offset Delta x_ref from the rear of the vehicle to it.

Writes multiball_4.png, multiball_8.png, multiball_16.png (one panel each) and
multiball_combined.png (the three side by side).
"""

import os
from pathlib import Path

import numpy as np

import matplotlib as mpl
mpl.rcParams.update({
    'font.size': 18,
    'axes.titlesize': 24,
    'axes.labelsize': 20,
    'xtick.labelsize': 16,
    'ytick.labelsize': 16,
    'savefig.bbox': 'tight',
    'savefig.dpi': 200,
})
import matplotlib.pyplot as plt

# Output goes to the repository's figures/ (override with SHFM_FIGURE_DIR).
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = Path(os.environ.get("SHFM_FIGURE_DIR", os.path.join(_REPO, "figures")))
OUT_DIR.mkdir(parents=True, exist_ok=True)

# Vehicle dimensions, in metres.
rlength = 3.6490   # overall length
rwidth = 0.7       # width
offset = np.array([0.4845, 0])  # rear axle centre, measured from the rear

N_VALUES = (4, 8, 16)


def calculate_balls_positions(rlength, offset, n):
    """Centres of the n discs, evenly spread along the length."""
    return np.array([(rlength / (n + 1) * (i + 1), 0) for i in range(n)]) - offset


def calculate_min_radius(rwidth, x_ball_0, offset):
    """Smallest radius covering a rectangle of width rwidth."""
    return np.sqrt((rwidth / 2) ** 2 + (x_ball_0 + offset[0]) ** 2)


def annotate_reference(ax):
    """Reference point O_R and the offset Delta x_ref from the rear to O_R.

    The vertical axis carries the length and its origin is O_R, the centre of
    the rear axle, so the rear of the vehicle is at y = -offset.
    """
    rear = -offset[0]
    x_dim = -rwidth / 2 - 0.14          # dimension line, left of the body
    # extension lines: rear of the vehicle and level of O_R
    for y in (rear, 0.0):
        ax.plot([-rwidth / 2, x_dim - 0.06], [y, y], color='k',
                linewidth=1.0, zorder=5)
    ax.annotate('', xy=(x_dim, 0.0), xytext=(x_dim, rear),
                arrowprops=dict(arrowstyle='<->', color='k', linewidth=1.6,
                                shrinkA=0, shrinkB=0), zorder=6)
    ax.text(x_dim - 0.07, rear / 2, r'$\Delta x_{\mathrm{ref}}$',
            ha='right', va='center', fontsize=17, zorder=7,
            bbox=dict(facecolor='white', edgecolor='none', pad=1.0, alpha=0.9))
    # reference point O_R
    ax.plot(0.0, 0.0, 'o', color='red', markersize=9, zorder=8)
    ax.text(0.09, 0.0, r'$O_R$', color='red', ha='left', va='center',
            fontsize=17, zorder=8,
            bbox=dict(facecolor='white', edgecolor='none', pad=0.5, alpha=0.8))


def draw_covering(ax, n, xhalf=None):
    """Draw the vehicle rectangle and its n covering discs on ax.

    The length of the vehicle runs along the vertical axis.

    xhalf: half-width imposed on the x axis, so that panels share a scale.
    """
    centers = calculate_balls_positions(rlength, offset, n)
    radius = calculate_min_radius(rwidth, centers[0][0], offset)

    # vehicle rectangle (x = width, y = length)
    rect = plt.Rectangle(
        (-rwidth / 2, -offset[0]), rwidth, rlength,
        linewidth=3.0, edgecolor='r', facecolor='none', zorder=3,
    )
    ax.add_patch(rect)

    # covering discs
    for c in centers:
        circle = plt.Circle(
            (c[1], c[0]), radius,
            color='b', fill=False, linestyle='--', linewidth=2.0, zorder=2,
        )
        ax.add_artist(circle)
        ax.plot(c[1], c[0], 'b.', markersize=8, zorder=4)

    annotate_reference(ax)

    half = xhalf if xhalf is not None else radius + 0.3
    ax.set_xlim(-half, half)
    ax.set_ylim(-offset[0] - 0.5, rlength - offset[0] + 0.5)
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('Largeur (m)')
    ax.grid(True, linestyle=':', linewidth=0.6, alpha=0.7)


def plot_single(n):
    fig, ax = plt.subplots(figsize=(6, 9))
    draw_covering(ax, n)
    ax.set_ylabel('Longueur (m)')
    ax.set_title(f"$N_{{balles}} = {n}$")
    out = OUT_DIR / f'multiball_{n}.png'
    fig.savefig(out)
    plt.close(fig)
    print(f'  -> {out}')


def plot_combined():
    fig, axes = plt.subplots(1, 3, figsize=(15, 9), sharey=True)
    # common x range: that of the largest radius (fewest discs), plus a margin
    centers_min = calculate_balls_positions(rlength, offset, min(N_VALUES))
    xhalf = calculate_min_radius(rwidth, centers_min[0][0], offset) + 0.3
    for ax, n in zip(axes, N_VALUES):
        draw_covering(ax, n, xhalf=xhalf)
        ax.set_title(f"$N_{{balles}} = {n}$")
    axes[0].set_ylabel('Longueur (m)')
    fig.tight_layout()
    out = OUT_DIR / 'multiball_combined.png'
    fig.savefig(out)
    plt.close(fig)
    print(f'  -> {out}')


if __name__ == '__main__':
    for n in N_VALUES:
        plot_single(n)
    plot_combined()
