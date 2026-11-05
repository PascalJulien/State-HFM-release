"""Drawing of HFM paths in the style of the Bench-MR (mpb) plates.

Grey map, path line, dashed vehicle footprints (about ``n_robots`` of them
along the path), axes in grid units with their equivalent in metres (one unit
is ``1/zoom`` m, since the vehicle of length ``bot_dim[0]`` m spans
``bot_dim[0] * zoom`` grid units).

:func:`set_output` and :func:`redirect` decide where the plates are written;
:mod:`shfm.plates` and the plotting tools of Bench-MR call them.

Depends on numpy and matplotlib only.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Colour cycle; the first path is blue.
_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple',
           'tab:brown', 'tab:olive', 'tab:cyan']


def _np(a):
    return a.get() if hasattr(a, 'get') else np.asarray(a)


# Output directory, geodesic cache and scenario description; see set_output().
OUT_DIR = 'images_viz'        # figures
CACHE_DIR = 'hfm_cache'       # geodesics (.npz)
CONDITIONS = ''               # description of the scenario


def set_output(out_dir=None, cache_dir=None, conditions=None):
    """Set the output directory, the geodesic cache and the scenario description."""
    global OUT_DIR, CACHE_DIR, CONDITIONS
    if out_dir is not None:
        OUT_DIR = out_dir
        import os; os.makedirs(out_dir, exist_ok=True)
    if cache_dir is not None:
        CACHE_DIR = cache_dir
        import os; os.makedirs(cache_dir, exist_ok=True)
    if conditions is not None:
        CONDITIONS = conditions


def redirect(save_file):
    """``save_file`` moved into ``OUT_DIR``: its base name, joined to ``OUT_DIR``."""
    import os
    if save_file is None:
        return None
    base = os.path.basename(save_file)
    return os.path.join(OUT_DIR, base)


# Saving and reloading geodesics, freeing GPU memory.
_GEO_COUNTER = [0]


def save_geodesics(geos, out_dir=None, name=None, tag=None):
    """Save the geodesics, as CPU arrays, in ``out_dir`` (``CACHE_DIR`` by
    default), to reload them later without running the solver. ``tag`` is
    appended to the generated name (for instance ``'dep30-70_arr90-80'``)."""
    import os
    if out_dir is None:
        out_dir = CACHE_DIR
    os.makedirs(out_dir, exist_ok=True)
    arrs = {f'geo_{i}': _np(g) for i, g in enumerate(geos)}
    if name is None:
        name = f'hfmout_{_GEO_COUNTER[0]:03d}'
        _GEO_COUNTER[0] += 1
        if tag:
            name += '_' + str(tag)
    path = os.path.join(out_dir, name + '.npz')
    np.savez_compressed(path, **arrs)
    return path


def save_current_figure(name_or_npz_path):
    """Save the current figure in ``OUT_DIR``. Takes a name, or the path of a
    ``.npz`` whose base name is reused with ``.png``, so that a figure and its
    geodesics share a name."""
    import os
    import matplotlib.pyplot as plt
    base = os.path.basename(str(name_or_npz_path))
    if base.endswith('.npz'):
        base = base[:-4] + '.png'
    elif not base.lower().endswith(('.png', '.pdf')):
        base = base + '.png'
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, base)
    plt.savefig(out, bbox_inches='tight', dpi=130)
    return out


def load_geodesics(name, out_dir=None):
    """Reload saved geodesics. ``name`` is an index or a file name."""
    import os
    if out_dir is None:
        out_dir = CACHE_DIR
    if isinstance(name, int):
        name = f'hfmout_{name:03d}'
    z = np.load(os.path.join(out_dir, name + '.npz'))
    return [z[k] for k in sorted(z.keys(), key=lambda s: int(s.split('_')[1]))]


def free_hfmout_gpu(hfmOut, keep=('geodesics',)):
    """Release the large GPU arrays of ``hfmOut`` (value function, flow...),
    keeping ``keep`` (the geodesics, which are small), then empty the cupy
    memory pool."""
    try:
        for k in list(hfmOut.keys()):
            if k not in keep:
                try:
                    hfmOut[k] = None
                except Exception:
                    pass
    except Exception:
        pass
    try:
        import gc, cupy
        gc.collect()
        cupy.get_default_memory_pool().free_all_blocks()
    except Exception:
        pass


def gpu_free_gb():
    """Free GPU memory in GB, read through cupy; None without cupy or GPU."""
    try:
        import cupy
        free, _total = cupy.cuda.Device().mem_info
        return free / 1e9
    except Exception:
        return None


def free_gpu(*hfmOuts, keep=('geodesics', 'geodesic_states',
                             'geodesic_stopping_criteria')):
    """Free GPU memory at the end of a loop iteration: release the large
    arrays of the given ``hfmOut`` (except ``keep``), close the matplotlib
    figures and return the blocks to the cupy pool. Call it after ``savefig``."""
    for h in hfmOuts:
        free_hfmout_gpu(h, keep=keep)
    try:
        import matplotlib.pyplot as plt
        plt.close('all')
    except Exception:
        pass
    try:
        import gc, cupy
        gc.collect()
        cupy.get_default_memory_pool().free_all_blocks()
    except Exception:
        pass


def geodesics_status(geos):
    """``(n_valid, n_total)``: a geodesic is valid when it is 2-D with at
    least 2 points. ``(0, n)`` with ``n > 0`` means that all of them are
    degenerate, typically because the GPU was saturated during the solve."""
    n = 0
    for g in geos:
        a = _np(g)
        if getattr(a, 'ndim', 0) == 2 and min(a.shape) >= 2:
            n += 1
    return n, len(geos)


def warn_if_degenerate(geos, where='run_controls'):
    """Warn when every geodesic is degenerate, which leads to an empty
    figure. Returns the number of valid geodesics."""
    nok, ntot = geodesics_status(geos)
    if nok == 0 and ntot > 0:
        free = gpu_free_gb()
        fs = f'{free:.2f} GB' if free is not None else '?'
        print(f"warning: {where}: all {ntot} geodesics are degenerate "
              f"(GPU free ~{fs}); the figure is probably empty: free the GPU and run again.")
    return nok


def geo_label(env, method, run=None, **params):
    """Deterministic label of an HFM solve, for the cache of :func:`cached_solve`.
    For instance ``geo_label('shortcut', '4WS', run=0, k1=10, k2=50, nu_FWD=3)``
    gives ``'shortcut__4WS__run0__k1=10__k2=50__nu_FWD=3'``."""
    parts = [str(env), str(method)]
    if run is not None:
        parts.append('run%s' % run)
    for k in sorted(params):
        v = params[k]
        parts.append('%s=%s' % (k, ('%g' % v) if isinstance(v, float) else v))
    key = '__'.join(parts)
    for ch in ' /\\:':
        key = key.replace(ch, '-')
    return key


def cached_solve(label, solve_fn, cache_dir='hfm_cache/labeled'):
    """Labelled cache of HFM solves. When a geodesic is stored under
    ``label``, load it (on the CPU) and return it; otherwise call
    ``solve_fn()``, which solves on the GPU and returns the ``(4, N)``
    geodesic, store the result under ``label`` and return it.

    Usage: ``cached_solve(geo_label('shortcut', '4WS', run=0, **cfg),
    lambda: ...)``.
    """
    import os
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, label + '.npz')
    if os.path.exists(path):
        z = np.load(path, allow_pickle=False)
        return z['geo']
    geo = solve_fn()
    g = _np(geo)
    np.savez_compressed(path, geo=g.astype(np.float32))
    return g


def robot_corners(bot_dim=(3.6490, 0.7000), offset=(0.4845, 0.0), zoom=4):
    """Corners of the vehicle rectangle, in the vehicle frame with the
    rear-axle centre at the origin, in grid units; the front is on negative
    ``x``, as in :meth:`shfm.vehicle.Vehicle.corners`."""
    Lf = bot_dim[0] * zoom - offset[0] * zoom   # front
    Lb = offset[0] * zoom                         # rear
    w = 0.5 * bot_dim[1] * zoom                   # half width
    return np.array([[-Lf, -w], [-Lf, w], [Lb, w], [Lb, -w]])


def meters_per_unit(bot_dim=(3.6490, 0.7000), zoom=4):
    """One grid unit in metres: the vehicle of length ``bot_dim[0]`` m spans
    ``bot_dim[0] * zoom`` units, so one unit is ``1/zoom`` m."""
    return 1.0 / float(zoom)


def _axis_labels(ax, zoom, bot_dim, label_fs=18, tick_fs=14):
    mpu = meters_per_unit(bot_dim, zoom)
    note = f'1 unit = {mpu:.3g} m'
    ax.set_xlabel(f'$x$ (units ; {note})', fontsize=label_fs)
    ax.set_ylabel(f'$y$ (units ; {note})', fontsize=label_fs)
    ax.tick_params(labelsize=tick_fs)


def draw_controls_mpb(ax, geos, walls, X, zoom=4,
                      bot_dim=(3.6490, 0.7000), offset=(0.4845, 0.0),
                      colors=None, n_robots=18, seed=None, tips=None,
                      title=None, flip_theta=True, axislabels=True):
    """Draw the geodesics on ``ax`` in the style of the Bench-MR plates.

    geos : list of geodesics, ``(3, N)``, ``(4, N)`` or ``(N, 3)``. walls : 2-D
    boolean map. X : meshgrid (``X[0]`` is x, ``X[1]`` is y). zoom : zoom
    factor of the environment. flip_theta : adds pi to theta for drawing,
    since :func:`robot_corners` puts the front on negative ``x``.
    """
    corners = robot_corners(bot_dim, offset, zoom)
    Xx, Xy = _np(X[0]), _np(X[1])
    ax.set_aspect('equal')
    ax.pcolormesh(Xx, Xy, _np(walls).astype(float), cmap='Greys',
                  vmin=0.0, vmax=1.7, alpha=0.55, shading='auto')
    if colors is None:
        colors = _COLORS
    for k, geo in enumerate(geos):
        g = _np(geo)
        # HFM geodesics are (k, N) with k in {3, 4}; degenerate ones (fewer
        # than 2 points) are skipped.
        if getattr(g, 'ndim', 0) != 2 or min(g.shape) < 2:
            continue
        if g.shape[0] in (3, 4) and g.shape[0] <= g.shape[1]:
            x, y, th = g[0], g[1], g[2]
        elif g.shape[1] in (3, 4):
            x, y, th = g[:, 0], g[:, 1], g[:, 2]
        else:
            continue
        if len(x) < 2:
            continue
        col = colors[k % len(colors)]
        thd = th + np.pi if flip_theta else th
        ax.plot(x, y, color=col, lw=2.2, alpha=0.8, zorder=3)
        idxs = np.unique(np.linspace(0, len(x) - 1, max(2, n_robots)).astype(int))
        for i in idxs:
            c, s = np.cos(thd[i]), np.sin(thd[i])
            R = np.array([[c, s], [-s, c]])
            poly = corners @ R + np.array([x[i], y[i]])
            ax.add_patch(Polygon(poly, closed=True, fill=False, linestyle='--',
                                 edgecolor=col, lw=1.2, alpha=0.9, zorder=2))
    # Optional start (blue) and goals (red).
    if seed is not None:
        s = _np(seed)
        ax.plot(s[0], s[1], 'o', color='blue', ms=8, zorder=5)
    if tips is not None:
        for tip in tips:               # possibly a list of cupy arrays
            t = _np(tip)
            ax.plot(t[0], t[1], 'o', color='red', ms=8, zorder=5)
    if axislabels:
        _axis_labels(ax, zoom, bot_dim)
    if title is not None:
        ax.set_title(title, fontsize=22, pad=12)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(float(Xx.min()), float(Xx.max()))
    ax.set_ylim(float(Xy.min()), float(Xy.max()))
    return ax


def draw_overlay(ax, items, walls, X, zoom=4, bot_dim=(3.6490, 0.7000),
                 offset=(0.4845, 0.0), seed=None, tips=None, footprints=True,
                 n_robots=7, flip_theta=True, title=None, legend_title=None,
                 cmap='plasma', axislabels=True):
    """Overlay several paths on one axis, coloured by parameter value.

    items : list of ``(geo, label)``, ``geo`` being ``(3 or 4, N)`` or
    ``(N, 3 or 4)`` and ``label`` its legend entry. The colours are taken from
    the sequential colormap ``cmap``, in the order of the items.
    footprints : dashed vehicle footprints, in the same colours,
    semi-transparent.
    """
    corners = robot_corners(bot_dim, offset, zoom)
    Xx, Xy = _np(X[0]), _np(X[1])
    ax.set_aspect('equal')
    ax.pcolormesh(Xx, Xy, _np(walls).astype(float), cmap='Greys',
                  vmin=0.0, vmax=1.7, alpha=0.55, shading='auto')
    n = len(items)
    cm = plt.get_cmap(cmap)
    # spread over 85 % of the colormap, for contrast between successive values
    colors = [cm(0.0 + 0.85 * i / max(1, n - 1)) for i in range(n)]
    for k, (geo, label) in enumerate(items):
        g = _np(geo)
        if getattr(g, 'ndim', 0) != 2 or min(g.shape) < 2:
            continue
        if g.shape[0] in (3, 4) and g.shape[0] <= g.shape[1]:
            x, y, th = g[0], g[1], g[2]
        elif g.shape[1] in (3, 4):
            x, y, th = g[:, 0], g[:, 1], g[:, 2]
        else:
            continue
        col = colors[k]
        ax.plot(x, y, color=col, lw=2.6, alpha=0.95, zorder=3, label=label)
        if footprints:
            thd = th + np.pi if flip_theta else th
            idxs = np.unique(np.linspace(0, len(x) - 1, max(2, n_robots)).astype(int))
            for i in idxs:
                c, s = np.cos(thd[i]), np.sin(thd[i])
                R = np.array([[c, s], [-s, c]])
                poly = corners @ R + np.array([x[i], y[i]])
                ax.add_patch(Polygon(poly, closed=True, fill=False, linestyle='--',
                                     edgecolor=col, lw=0.9, alpha=0.55, zorder=2))
    if seed is not None:
        sd = _np(seed)
        ax.plot(sd[0], sd[1], 'o', color='blue', ms=9, zorder=6)
    if tips is not None:
        for tip in tips:
            t = _np(tip)
            ax.plot(t[0], t[1], 'o', color='red', ms=9, zorder=6)
    if axislabels:
        _axis_labels(ax, zoom, bot_dim)
    ax.legend(title=legend_title, fontsize=13, title_fontsize=14,
              loc='best', framealpha=0.92)
    if title is not None:
        ax.set_title(title, fontsize=20, pad=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(float(Xx.min()), float(Xx.max()))
    ax.set_ylim(float(Xy.min()), float(Xy.max()))
    return ax


def render_geo_mpb(walls, X, geos, out, color='tab:blue', n_robots=18, title='',
                   bot_dim=(3.6490, 0.7000), offset=(0.4845, 0.0), zoom=4,
                   figsize=(8, 8), dpi=150, seed=None, tips=None):
    """Standalone version: creates the figure and saves it."""
    fig, ax = plt.subplots(figsize=figsize)
    draw_controls_mpb(ax, geos, walls, X, zoom=zoom, bot_dim=bot_dim,
                      offset=offset, colors=[color] * 8, n_robots=n_robots,
                      seed=seed, tips=tips, title=title or None)
    plt.savefig(out, bbox_inches='tight', dpi=dpi)
    plt.close(fig)
