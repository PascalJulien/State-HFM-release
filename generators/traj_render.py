"""Rendering of scenes supplied by the caller: paths, navigation function,
control set.

The helpers do not depend on cupy or agd, use no global variables (the vehicle
dimensions are passed in ``robot``) and draw on the axes they are given.

Typical use::

    from traj_render import render_scene

    fig, ax = plt.subplots(figsize=(8, 8))
    render_scene(scene, ax=ax)
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow

PALETTE = (
    'tab:blue', 'tab:orange', 'tab:purple', 'tab:brown',
    'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan',
)


def show_vehicle(ax, x, y, theta, robot, color='blue'):
    """Arrow showing the vehicle and its heading, rear axle towards the front.

    ``robot``: dict with ``rlength``, ``offset`` (2,) and, optionally,
    ``zoom_factor``.
    """
    rlength = robot['rlength']
    offset = np.asarray(robot.get('offset', [0.0, 0.0]), dtype=float)
    zoom = float(robot.get('zoom_factor', 1.0))

    rlength_zoom = rlength * zoom
    offset_zoom = offset * zoom

    rot = np.array([[np.cos(theta), np.sin(theta)],
                    [-np.sin(theta), np.cos(theta)]])
    offset_theta = offset_zoom @ rot

    unit = np.array([np.cos(theta), np.sin(theta)])
    arrow_vec = rlength_zoom * unit - offset_theta

    ax.add_patch(FancyArrow(
        x - offset_theta[0], y - offset_theta[1],
        arrow_vec[0], arrow_vec[1],
        width=0.5 * zoom, length_includes_head=True,
        color=color, alpha=0.85,
    ))


def draw_bot(ax, gamma, robot, nb_pt=10, colors=('g', 'y', 'r'),
             state=None, ls='-'):
    """Draw the rectangular outline of the vehicle at ``nb_pt`` configurations.

    ``gamma``: ndarray ``(>= 3, N)``, rows ``x, y, theta, ...``.
    ``robot``: dict with ``rlength``, ``rwidth``, ``offset`` (2,),
    ``wheelbase`` and ``zoom_factor``.
    ``state``: optional ndarray ``(N,)``; when given, each outline is blue
    (forward, state below 1.5) or red (backward) instead of ``colors[1]``.
    ``ls``: line style of the intermediate outlines.
    """
    rlength = robot['rlength']
    rwidth = robot['rwidth']
    offset = np.asarray(robot.get('offset', [0.0, 0.0]), dtype=float)
    wheelbase = robot['wheelbase']
    zoom = float(robot.get('zoom_factor', 1.0))

    offset_zoom = offset * zoom
    bot_dim_zoom = np.array([rlength, rwidth]) * zoom
    wheelbase_zoom = wheelbase * zoom

    rlength_z = bot_dim_zoom[0] - offset_zoom[0]
    rwidth_z = bot_dim_zoom[1]
    length_front = rlength_z
    length_back = offset_zoom[0]
    width_left = 0.5 * rwidth_z
    width_right = 0.5 * rwidth_z

    corners = np.array([
        -length_front - 1j * width_left,
        -length_front + 1j * width_right,
         length_back  - 1j * width_left,
         length_back  + 1j * width_right,
        -bot_dim_zoom[0] / 2 + 1j * width_left,
        -bot_dim_zoom[0] / 2 - 1j * width_right,
        -wheelbase_zoom + 0j,
    ])

    N = gamma.shape[1]
    rot = np.cos(gamma[2]) + 1j * np.sin(gamma[2])
    pos = gamma[0] + 1j * gamma[1]
    pt = corners[:, None] * rot[None, :] + pos[None, :]   # (7, N) complex

    if nb_pt == 0:
        return
    indices = np.linspace(0, N - 1, nb_pt).astype(int)

    def _plot_outline(c, color):
        ax.plot(np.real([pt[0, c], pt[1, c]]), np.imag([pt[0, c], pt[1, c]]), color, linestyle=ls)
        ax.plot(np.real([pt[2, c], pt[3, c]]), np.imag([pt[2, c], pt[3, c]]), color, linestyle=ls)
        ax.plot(np.real([pt[1, c], pt[3, c]]), np.imag([pt[1, c], pt[3, c]]), color, linestyle=ls)
        ax.plot(np.real([pt[0, c], pt[2, c]]), np.imag([pt[0, c], pt[2, c]]), color, linestyle=ls)
        ax.plot(np.real([pt[4, c], pt[5, c]]), np.imag([pt[4, c], pt[5, c]]), color, linestyle=ls)

    for c in indices:
        if state is not None:
            col = 'tab:blue' if state[c] < 1.5 else 'tab:red'
        else:
            col = colors[1]
        _plot_outline(c, col)

    if state is None:
        _plot_outline(0, colors[0])
        _plot_outline(N - 1, colors[2])


def render_navfunc(scene, ax=None, title=None, cmap='jet'):
    """3D scatter of the navigation function U(q) over (x, y, theta).

    Draws ``4WS_nav_function.png``: a 3D scatter coloured by U, the
    configurations in collision masked through ``walls_3d``.

    Points are drawn only where the configuration is valid (``walls_3d == 0``,
    or everywhere without ``walls_3d``). The grid is subsampled, so that a
    dense grid does not give a cloud of millions of points.
    """
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')

    U = scene['U']
    nx, ny, nt = U.shape

    if 'X_x' in scene:
        # X_x, X_y and X_theta may be stored as 1-D axes or as full meshgrids;
        # the 1-D axis is extracted in both cases.
        def _axis(arr, axis_id, n):
            a = np.asarray(arr)
            if a.ndim == 1:
                return a if a.size == n else np.linspace(a.min(), a.max(), n)
            slicer = [0] * a.ndim
            slicer[axis_id] = slice(None)
            return a[tuple(slicer)]

        xs = _axis(scene['X_x'], 0, nx)
        ys = _axis(scene['X_y'], 1, ny)
        ts = _axis(scene['X_theta'], 2 if scene['X_theta'].ndim > 1 else 0, nt)
    else:
        xs = np.arange(nx)
        ys = np.arange(ny)
        ts = np.arange(nt)

    # One point in two along x and y, every heading. Besides thinning the
    # cloud, the subsampling sets the range of the colour scale.
    sx = sy = 2
    st = 1

    xs_s, ys_s, ts_s = xs[::sx], ys[::sy], ts[::st]
    U_s = U[::sx, ::sy, ::st]
    walls = scene.get('walls_3d')
    if walls is not None:
        walls_s = walls[::sx, ::sy, ::st]
        valid_s = (walls_s == 0) & np.isfinite(U_s)
    else:
        valid_s = np.isfinite(U_s)

    # The points are placed at their indices in the subsampled array, not at
    # their coordinates, and the axes are bounded by the shape of that array.
    Xg, Yg, Tg = np.meshgrid(np.arange(U_s.shape[0]), np.arange(U_s.shape[1]),
                             np.arange(U_s.shape[2]), indexing='ij')
    pts_x = Xg[valid_s].ravel()
    pts_y = Yg[valid_s].ravel()
    pts_t = Tg[valid_s].ravel()
    pts_u = U_s[valid_s].ravel()

    # s=100: at this density the default marker leaves a gap of about 1.5 px
    # between rows, which shows as stripes.
    sc = ax.scatter(pts_x, pts_y, pts_t, c=pts_u, cmap=cmap, marker='s',
                    alpha=0.5, s=100)
    plt.colorbar(sc)
    ax.xaxis.set_ticklabels([])
    ax.yaxis.set_ticklabels([])
    ax.zaxis.set_ticklabels([])
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('\u0398 Axis')
    ax.set_xlim(0, U_s.shape[0])
    ax.set_ylim(0, U_s.shape[1])
    ax.set_zlim(0, U_s.shape[2])
    ax.view_init(elev=44, azim=-65)
    if title:
        ax.set_title(title)
    return ax


def render_controlset(scene, ax=None, title=None,
                      show_offsets=False, control_color='red'):
    """The 4WS control set: the grid as a scatter, the controls as arrows.

    The controls are drawn in ``control_color``; with ``show_offsets=True``,
    the offsets are added as black arrows.
    """
    if ax is None:
        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(111, projection='3d')

    grid = scene['grid_points']
    controls = scene['controls']
    offsets = scene.get('offsets')
    meta = scene.get('meta', {})

    ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2],
               color='gray', s=10, alpha=0.5)

    if show_offsets and offsets is not None:
        ax.quiver(offsets[:, 0], offsets[:, 1], offsets[:, 2],
                  offsets[:, 3], offsets[:, 4], offsets[:, 5],
                  color='black', length=1.5, normalize=False,
                  arrow_length_ratio=0.2, linewidth=1.2)

    ax.quiver(controls[:, 0], controls[:, 1], controls[:, 2],
              controls[:, 3], controls[:, 4], controls[:, 5],
              color=control_color, length=1.0, normalize=False,
              arrow_length_ratio=0.2, linewidth=1.5)

    center = meta.get('center', [0, 0, 0])
    ax.scatter([center[0]], [center[1]], [center[2]],
               color='blue', s=80, edgecolor='black', zorder=5)

    ax.set_xlabel(r'$\dot{x}$', fontsize=18, labelpad=10)
    ax.set_ylabel(r'$\dot{y}$', fontsize=18, labelpad=10)
    ax.set_zlabel(r'$\dot{\theta}$', fontsize=18, labelpad=10)
    ax.tick_params(labelsize=13)
    ax.set_title(title or meta.get('title', '4WS control set'),
                 fontsize=20, pad=12)
    # Frame centred on the control vectors, not on the whole theta grid.
    import numpy as _np
    span = float(_np.abs(controls[:, 3:6]).max()) * 1.2 if len(controls) else 3.0
    ax.set_xlim(-span, span)
    ax.set_ylim(-span, span)
    zmax = float(_np.abs(controls[:, 5]).max()) * 1.3 if len(controls) else 1.0
    ax.set_zlim(-max(zmax, 0.2), max(zmax, 0.2))
    ax.view_init(elev=22, azim=-60)
    return ax


def render_scene(scene, ax=None, title=None, traj_colors=PALETTE,
                 n_bot_samples=200, show_front_wheel=True, state_colors=False):
    """Draw a path figure from a scene:

        - background ``pcolormesh(*X, walls, cmap='binary')``,
        - arrows for the seed (blue) and the tips (red), with their index,
        - trace of the rear axle (one colour per path) and scatter coloured by
          state,
        - trace of the front axle, dotted,
        - rectangular outlines of the vehicle at regular intervals.

    Parameters
    ----------
    scene : dict
        ``walls``, ``X``, ``seed``, ``tips``, ``geos``, ``robot``; built by the
        caller.
    ax : matplotlib Axes, optional
        Created when not given.
    title : str, optional
        Defaults to ``scene['meta'].get('title')``.
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 8))

    walls = scene['walls']
    X = scene['X']
    seed = scene['seed']
    tips = scene['tips']
    geos = scene['geos']
    robot = scene['robot']
    meta = scene.get('meta', {})

    ax.set_aspect('equal')
    ax.pcolormesh(X[0], X[1], walls, cmap='binary', shading='auto')

    show_vehicle(ax, seed[0], seed[1], seed[2], robot, color='blue')
    for i, tip in enumerate(tips):
        show_vehicle(ax, tip[0], tip[1], tip[2], robot, color='red')
        ax.text(tip[0], tip[1], str(i), fontsize=12,
                ha='left', va='bottom', color='red')

    wheelbase_zoom = robot['wheelbase'] * float(robot.get('zoom_factor', 1.0))

    for k, geo in enumerate(geos):
        if state_colors and geo.shape[0] >= 4:
            # Forward (blue) and backward (red) travel, read from the state
            # geo[3]: 0 and 1 forward, 2 and 3 backward. The geodesic runs from
            # the goal to the start; it is reversed to read from start to goal.
            geo = geo[:, ::-1]
            state = geo[3]
            forward = state < 1.5
            idx = np.flatnonzero(np.diff(forward.astype(int))) + 1
            for seg in np.split(np.arange(geo.shape[1]), idx):
                if len(seg) < 2:
                    continue
                col = 'tab:blue' if forward[seg[0]] else 'tab:red'
                ax.plot(geo[0, seg], geo[1, seg], color=col, linewidth=1.8)
            geo_print = geo.copy()
            geo_print[2] = geo_print[2] + np.pi
            N = geo.shape[1]
            if N > 0 and n_bot_samples > 0:
                step_count = max(1, int(N / n_bot_samples))
                draw_bot(ax, geo_print, robot, nb_pt=step_count,
                         state=state, ls='--')
            continue

        color = traj_colors[k % len(traj_colors)]
        ax.plot(geo[0], geo[1], color=color, linewidth=1.6,
                label=f'backward {k}')
        if geo.shape[0] >= 4:
            ax.scatter(geo[0], geo[1], c=geo[3], s=4, cmap='viridis')
        if show_front_wheel:
            ax.plot(
                geo[0] + wheelbase_zoom * np.cos(geo[2]),
                geo[1] + wheelbase_zoom * np.sin(geo[2]),
                linestyle=':', color=color, linewidth=1.0,
            )
        geo_print = geo.copy()
        geo_print[2] = geo_print[2] + np.pi
        N = geo.shape[1]
        if N > 0 and n_bot_samples > 0:
            # nb_pt is the number of outlines, not a step. Olive, since blue is
            # the colour of SHFM 2WS in the benchmark figures.
            draw_bot(ax, geo_print, robot,
                     nb_pt=n_bot_samples,
                     colors=('g', 'olive', 'r'))

    ax.set_title(title or meta.get('title', ''))
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    return ax
