#!/usr/bin/env python3
"""The four 4WS control vectors and their adaptive stencils.

Each control vector u is one edge of the 4WS indicatrix (the same kinematics as
``tissot.py``). Its relaxed metric M = n n^T + eps^2 (I - n n^T), with
n = u / |u|, is decomposed by Selling's algorithm into weights and integer
offsets (the Voronoi decomposition). Drawn in an (x, y, theta) box with the
integer points of Z^3, q at the origin, the controls in red and the offsets in
blue from q, their thickness growing with the weight.

(a) stencil_3d_a_darker_grid: the four controls alone.
(b) stencil_3d_b_darker_grid: the four controls and their offsets g.
Each is written as .png and .pdf.
"""
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib as mpl
mpl.rcParams['font.family'] = 'DejaVu Sans'
mpl.rcParams['savefig.dpi'] = 200
mpl.rcParams['savefig.bbox'] = 'tight'
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from pathlib import Path

import os as _os
_REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
OUT = Path(_os.environ.get("SHFM_FIGURE_DIR", _os.path.join(_REPO, "figures")))
OUT.mkdir(parents=True, exist_ok=True)
LIM = 3.5
EPS = 0.1

# Selling decomposition.
def selling_algorithm(M, max_iter=100):
    e0 = np.array([1, 0, 0]); e1 = np.array([0, 1, 0])
    e2 = np.array([0, 0, 1]); e3 = -(e0 + e1 + e2)
    sb = [e0, e1, e2, e3]
    for _ in range(max_iter):
        obtuse = True; mp = -np.inf; bi, bj = 0, 1
        for i in range(4):
            for j in range(i + 1, 4):
                p = np.dot(sb[i], M @ sb[j])
                if p > 1e-10:
                    obtuse = False
                    if p > mp:
                        mp = p; bi, bj = i, j
        if obtuse:
            break
        i, j = bi, bj; new = sb.copy(); new[i] = -sb[i]
        for k in range(4):
            if k != i and k != j:
                new[k] = sb[k] + sb[i]
        sb = new
    return sb

def voronoi_decomposition(n_vec, epsilon):
    n = n_vec / np.linalg.norm(n_vec)
    M = np.outer(n, n) + epsilon**2 * (np.eye(3) - np.outer(n, n))
    sb = selling_algorithm(M)
    offs, wts = [], []
    for i in range(4):
        for j in range(i + 1, 4):
            rem = [k for k in range(4) if k not in (i, j)]; k, l = rem
            v = np.cross(sb[k], sb[l])
            if np.linalg.norm(v) > 1e-10:
                g = np.gcd.reduce(v.astype(int))
                if g != 0:
                    v = np.round(v / g).astype(int)
                w = -np.dot(sb[i], M @ sb[j])
                if abs(w) > 1e-10:
                    offs.append(v); wts.append(abs(w))
    # orient every offset so that <n, g> >= 0
    out = []
    for o, w in zip(offs, wts):
        if np.dot(n, o) < 0:
            o = -o
        out.append((np.asarray(o, float), w))
    return out

# The four 4WS control vectors.
psi_max, phi_max, wb = 0.5, 0.5, 1.38
combos = [(phi_max, psi_max), (phi_max, -psi_max),
          (-phi_max, -psi_max), (-phi_max, psi_max)]
def cv(p, s):
    # cos(s): rolling without slip, speed of the rear axle centre.
    c = np.cos(s)
    return np.array([np.cos(s), np.sin(s), c * (np.tan(p) - np.tan(s)) / wb])
U = [cv(p, s) for (p, s) in combos]
STENCILS = [voronoi_decomposition(u, EPS) for u in U]

# Drawing.
def grid_q(ax):
    c = np.arange(int(np.floor(-LIM)) + 1, int(np.ceil(LIM)))
    xx, yy, zz = np.meshgrid(c, c, c)
    ax.scatter(xx, yy, zz, s=6, color='lightgray', alpha=0.9)
    ax.scatter([0], [0], [0], color='black', s=25)
    ax.text(-0.45, -0.30, 0.05, 'q', color='black', fontsize=15, ha='right', va='top')

def vec(ax, p, color, lw, label=None, fs=12, alpha=1.0):
    p = np.asarray(p, float)
    ax.quiver(0, 0, 0, p[0], p[1], p[2], color=color, linewidth=lw,
              arrow_length_ratio=0.16, alpha=alpha)
    if label:
        e = 0.30 / max(np.linalg.norm(p), 1e-9)
        ax.text(p[0] * (1 + e), p[1] * (1 + e), p[2] * (1 + e), label,
                color=color, fontsize=fs)

# View angles of the two panels (elevation, azimuth), one per panel.
VIEWS = {"a": (16.5, -97.6), "b": (18.5, -95.7)}


def finish(ax, view="a"):
    ax.set_xlim(-LIM, LIM); ax.set_ylim(-LIM, LIM); ax.set_zlim(-LIM, LIM)
    ax.set_xlabel('x', fontsize=16, labelpad=8)
    ax.set_ylabel('y', fontsize=16, labelpad=8)
    ax.set_zlabel(r'$\theta$', fontsize=16, labelpad=10)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(*VIEWS[view])
    ax.tick_params(labelsize=9)

# --- (a) the four controls alone ---------------------------------------------
figa = plt.figure(figsize=(7, 7)); axa = figa.add_subplot(111, projection='3d')
grid_q(axa)
for i, u in enumerate(U):
    vec(axa, 2.5 * u, 'red', 2.3, r'$u_%d$' % (i + 1))
finish(axa)
axa.legend(handles=[Line2D([0], [0], color='red', lw=2.3,
           label="contrôles $u_i$ (arêtes de l'indicatrice 4WS)")],
           loc='upper left', fontsize=13)
figa.tight_layout()
figa.savefig(OUT / 'stencil_3d_a_darker_grid.png'); figa.savefig(OUT / 'stencil_3d_a_darker_grid.pdf')
print('OK (a)')

# --- (b) the four controls and their offsets ---------------------------------
figb = plt.figure(figsize=(7.5, 7.5)); axb = figb.add_subplot(111, projection='3d')
grid_q(axb)
wmax = max(w for st in STENCILS for (_, w) in st)
for i, (u, st) in enumerate(zip(U, STENCILS)):
    vec(axb, 2.5 * u, 'red', 2.2, r'$u_%d$' % (i + 1))
    for (g, w) in st:
        lw = 0.7 + 2.3 * (w / wmax)        # thickness grows with the weight
        vec(axb, g, 'royalblue', lw, alpha=0.9)
        axb.scatter([g[0]], [g[1]], [g[2]], s=18 + 60 * (w / wmax),
                    color='royalblue', alpha=0.9)
finish(axb, "b")
axb.legend(handles=[
    Line2D([0], [0], color='red', lw=2.2, label='contrôles $u_i$'),
    Line2D([0], [0], color='royalblue', lw=2.0,
           label="décalages $g_{ij}$ (réduction de Selling, épaisseur $\\propto \\rho_{ij}$)"),
], loc='upper left', fontsize=12)
figb.tight_layout()
figb.savefig(OUT / 'stencil_3d_b_darker_grid.png'); figb.savefig(OUT / 'stencil_3d_b_darker_grid.pdf')
print('OK (b)')

plt.show()