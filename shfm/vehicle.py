"""Geometry and footprint of the 4WS forklift.

All lengths are given in metres. The planner works on a grid whose scale is set
by ``zoom``: one metre of the real vehicle spans ``zoom`` grid units, so one
grid unit is ``1/zoom`` metres. Every function below that returns grid
quantities takes ``zoom`` explicitly, so that the same vehicle can be planned
on coarse or fine grids without modifying the model.
"""
import numpy as np

__all__ = ["Vehicle", "DEFAULT_VEHICLE"]


class Vehicle:
    """Dimensions of the forklift and the derived collision model.

    Parameters
    ----------
    length, width : float
        Total footprint of the vehicle, forks included, in metres.
    wheelbase : float
        Distance between the front and rear axles, in metres.
    rear_axle_offset : float
        Distance from the rear of the vehicle to the centre of the rear axle,
        in metres. The planner uses that centre as the reference point, so the
        footprint is not centred on it.
    n_balls : int
        Number of discs used to cover the footprint for collision checking.
    """

    def __init__(self, length=3.6490, width=0.7000, wheelbase=1.38,
                 rear_axle_offset=0.4845, n_balls=12):
        self.length = float(length)
        self.width = float(width)
        self.wheelbase = float(wheelbase)
        self.rear_axle_offset = float(rear_axle_offset)
        self.n_balls = int(n_balls)

    # -- grid conversions ---------------------------------------------------

    def metres_per_unit(self, zoom):
        """One grid unit, expressed in metres."""
        return 1.0 / float(zoom)

    def footprint_area(self):
        """Floor area of the vehicle, in square metres.

        Used as the reference scale when reading swept-area figures: a swept
        area of 60 m2 means roughly 23 times the area the vehicle itself
        occupies.
        """
        return self.length * self.width

    def corners(self, zoom):
        """Footprint corners in the vehicle frame, in grid units.

        The origin is the centre of the rear axle. The vehicle extends
        ``rear_axle_offset`` behind it and the rest in front, with the front
        on negative ``x``: these corners face ``theta + pi``, which is why
        :meth:`footprint_polygon` and :func:`shfm.metrics.swept_area` add pi
        to the heading.
        """
        front = (self.length - self.rear_axle_offset) * zoom
        back = self.rear_axle_offset * zoom
        half_w = 0.5 * self.width * zoom
        return np.array([[-front, -half_w], [-front, half_w],
                         [back, half_w], [back, -half_w]])

    def footprint_polygon(self, x, y, theta, zoom, flip_theta=True):
        """Footprint polygon at pose ``(x, y, theta)``, in grid units.

        ``theta`` is the heading of the vehicle, the direction its front
        points, for a path from the solver or from Bench-MR alike: reversing
        the order of a path does not change the heading of its poses.
        ``flip_theta`` adds pi to it, which compensates for :meth:`corners`
        placing the front on negative ``x``; leave it on to draw the vehicle
        facing ``theta``.
        """
        th = theta + np.pi if flip_theta else theta
        c, s = np.cos(th), np.sin(th)
        rot = np.array([[c, s], [-s, c]])
        return self.corners(zoom) @ rot + np.array([x, y])

    # -- multi-ball collision model ----------------------------------------

    def ball_centres(self, zoom):
        """Longitudinal positions of the covering discs, in grid units.

        The discs are spaced regularly along the vehicle axis, expressed
        relative to the rear-axle centre.
        """
        length_u = self.length * zoom
        offset_u = self.rear_axle_offset * zoom
        step = length_u / (self.n_balls + 1)
        return np.array([step * (i + 1) - offset_u for i in range(self.n_balls)])

    def ball_radius(self, zoom):
        """Radius of the covering discs, in grid units.

        ``hypot(width / 2, step)``, where ``step`` is the spacing between
        consecutive disc centres. The outermost centres sit one full ``step``
        from each end of the vehicle (see :meth:`ball_centres`), so this is the
        smallest radius whose discs reach the four corners: the covering is
        tight at the corners and larger than needed in between, where
        ``hypot(width / 2, step / 2)`` would suffice. The discs also reach
        ``radius - width / 2`` past each side and ``radius - step`` past each
        end, so the model never misses a collision and slightly overestimates
        the vehicle in narrow passages.
        """
        half_w = 0.5 * self.width * zoom
        step = (self.length * zoom) / (self.n_balls + 1)
        return float(np.hypot(half_w, step))


    # -- export -------------------------------------------------------------

    def to_svg(self, path):
        """Write the footprint as an SVG polygon, in metres.

        The rear-axle centre sits at the origin of the file, so a planner that
        takes a reference point of ``(0, 0)`` and a scale equal to the grid
        zoom places the vehicle correctly.
        """
        front = self.length - self.rear_axle_offset
        back = -self.rear_axle_offset
        half_w = 0.5 * self.width
        d = ("M %g %g H %g V %g H %g Z"
             % (back, half_w, front, -half_w, back))
        svg = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<svg xmlns="http://www.w3.org/2000/svg" version="1.1"\n'
            '     width="%gmm" height="%gmm" viewBox="%g %g %g %g">\n'
            '  <!-- Footprint in metres. Origin = centre of the rear axle. -->\n'
            '  <path\n'
            'd="%s" />\n'
            '</svg>\n'
            % (self.length, self.width, back, -half_w, self.length,
               self.width, d))
        with open(path, "w") as f:
            f.write(svg)
        return path



DEFAULT_VEHICLE = Vehicle()
"""The forklift this package is built around: an Adap2E fitted with forks."""
