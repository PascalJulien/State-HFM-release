"""Environments: occupancy maps and the lift to configuration space.

The planner does not work in the plane: a forklift is long and narrow, so
whether a position is free depends on the heading of the vehicle. The workspace
map is therefore lifted from two dimensions to three, ``(x, y, theta)``, by
sweeping the vehicle footprint over every sampled orientation.

The footprint is approximated by a chain of overlapping discs
(:class:`shfm.vehicle.Vehicle`). A configuration is in collision as soon as one
disc overlaps an obstacle, which reduces the test to a lookup in the distance
transform of the map, shifted along the vehicle axis.
"""
import os

import numpy as np
from scipy import ndimage

from .vehicle import DEFAULT_VEHICLE

__all__ = ["Environment", "lift_to_configuration_space",
           "from_occupancy", "from_image", "from_benchmark_run"]


class Environment:
    """An occupancy map together with its lift to configuration space.

    Attributes
    ----------
    walls : ndarray ``(nx, ny)`` of bool
        Workspace obstacles.
    walls3 : ndarray ``(nx, ny, n_theta)`` of bool
        Configurations in collision.
    walls3_dist : ndarray ``(nx, ny, n_theta)``
        Distance from each free configuration to the nearest obstacle, in grid
        units. Distances are purely spatial: the orientation axis carries an
        effectively infinite sampling step, so each distance is measured in the
        plane, to the nearest configuration in collision at the same heading.
    walls3_dist_saturated : ndarray
        Same, capped at ``saturation_cap`` (the vehicle length by default).
        Feeding the capped version to the solver stops the path from drifting
        to the centre of large open areas, while keeping the clearance
        behaviour in narrow passages.
    """

    def __init__(self, walls, X, walls_dist, walls3, walls3_dist,
                 walls3_dist_saturated, dxy, zoom, theta, vehicle=None, name=None):
        self.walls = walls
        self.X = X
        self.walls_dist = walls_dist
        self.walls3 = walls3
        self.walls3_dist = walls3_dist
        self.walls3_dist_saturated = walls3_dist_saturated
        self.dxy = dxy
        self.zoom = zoom
        self.theta = theta
        self.vehicle = vehicle or DEFAULT_VEHICLE
        self.name = name

    @property
    def shape(self):
        return self.walls.shape

    @property
    def n_theta(self):
        return len(self.theta)

    def metres_per_unit(self):
        return 1.0 / float(self.zoom)

    def __repr__(self):
        return ("Environment(name=%r, shape=%s, n_theta=%d, zoom=%s, "
                "1 unit = %.3g m)" % (self.name, self.shape, self.n_theta,
                                      self.zoom, self.metres_per_unit()))


def lift_to_configuration_space(walls_dist, theta, zoom, dxy, vehicle=None,
                                saturation_cap=None):
    """Sweep the disc-covered footprint over every orientation.

    For each disc centre, the distance map is shifted by the disc offset along
    the vehicle axis; a configuration is in collision when the shifted distance,
    or the distance at the reference point, is at most the disc radius.

    ``saturation_cap`` is the value, in metres, at which the clearance is
    capped. The default is the vehicle's full length, the cap of the benchmark
    comparisons.

    Returns
    -------
    (walls3, walls3_dist, walls3_dist_saturated)
    """
    vehicle = vehicle or DEFAULT_VEHICLE
    centres = vehicle.ball_centres(zoom)
    radius = vehicle.ball_radius(zoom)

    combined = np.zeros((len(theta),) + walls_dist.shape)
    for centre in centres:
        shifted = [ndimage.shift(walls_dist,
                                 -centre * np.array([np.cos(t), np.sin(t)]) / dxy[0],
                                 order=1)
                   for t in theta]
        combined += [np.logical_or(walls_dist <= radius, s <= radius)
                     for s in shifted]

    walls3 = np.stack(list(combined), axis=-1).astype(bool)
    free = np.logical_not(walls3)
    # The large third sampling step keeps the orientation axis out of the
    # distances, which stay purely spatial.
    dist = ndimage.distance_transform_edt(free, (dxy[0], dxy[1], 1e10))
    cap = (vehicle.length if saturation_cap is None else saturation_cap) * zoom
    return walls3, dist, np.minimum(dist, cap)


def _distance_transform_2d(walls, dx):
    return ndimage.distance_transform_edt(np.logical_not(walls), sampling=dx)


def from_occupancy(walls, zoom, n_theta=100, dxy=None, vehicle=None, name=None,
                   saturation_cap=None):
    """Build an :class:`Environment` from a boolean occupancy array."""
    walls = np.asarray(walls, dtype=bool)
    dxy = dxy or [1.0, 1.0]
    theta = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    X = np.meshgrid(np.arange(walls.shape[0]) * dxy[0],
                    np.arange(walls.shape[1]) * dxy[1], indexing="ij")
    walls_dist = _distance_transform_2d(walls, dxy[0])
    w3, d3, d3s = lift_to_configuration_space(walls_dist, theta, zoom, dxy,
                                             vehicle, saturation_cap)
    return Environment(walls, X, walls_dist, w3, d3, d3s, dxy, zoom, theta,
                       vehicle, name)


def from_image(path, target_shape, zoom, n_theta=100, vehicle=None, name=None,
               saturation_cap=None):
    """Build an environment from a binary image.

    Dark pixels are obstacles. The image is resampled to ``target_shape``,
    given as ``(nx, ny)``.

    Image files index rows then columns, with the first row at the top. The
    planner indexes ``x`` then ``y``, with ``y`` increasing upwards. The array
    is therefore transposed and flipped on load, and each axis is resampled
    independently.
    """
    import matplotlib.image as mpimg
    img = mpimg.imread(path)
    dark = img[:, :, 0] == 0 if img.ndim == 3 else img == 0
    walls = dark.T[:, ::-1]                       # (rows, cols) -> (x, y up)
    factors = (target_shape[0] / walls.shape[0],
               target_shape[1] / walls.shape[1])
    walls = ndimage.zoom(walls.astype(float), factors, order=1) > 0.5
    return from_occupancy(walls, zoom, n_theta, [1.0, 1.0], vehicle,
                          name or os.path.basename(path), saturation_cap)


def from_benchmark_run(run, zoom, n_theta=100, vehicle=None, name=None,
                       saturation_cap=None):
    """Build an environment from one run of a Bench-MR result file.

    Bench-MR stores the map as a flat string of ``0`` and ``1``, row by row from
    the bottom; this rebuilds the array, transposes it to the planner's
    ``(x, y)`` convention and lifts it. One Bench-MR cell is one grid unit on
    both sides, so the paths a run returns are already in the frame of the
    environment this builds, and need no rescaling before being measured.
    """
    env = run["environment"]
    width = env["width"]
    flat = env["map"]
    height = len(flat) // width
    walls = np.array([int(c) for c in flat]).reshape(height, width).T
    return from_occupancy(walls, zoom, n_theta, [1.0, 1.0], vehicle,
                          name or env.get("name"), saturation_cap)
