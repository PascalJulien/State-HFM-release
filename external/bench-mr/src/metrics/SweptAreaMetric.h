#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "TrajectoryMetric.h"

/**
 * Floor area swept by the robot along a trajectory.
 *
 * The union of the successive footprints, not the sum: overlapping poses are
 * counted once. Overshoots, crab motion, reversals and wide turns all raise it.
 *
 * The footprint is given in the robot frame, with the reference point at the
 * origin, in the same units as the path. The result is returned in square
 * metres, using the given number of metres per unit.
 */
class SweptAreaMetric {
 public:
  /**
   * @param path       poses as (x, y, theta), in grid units
   * @param footprint  polygon in the robot frame, reference point at origin
   * @param metres_per_unit  size of one grid unit, in metres
   * @param supersample  raster cells per grid unit; 4 is a good compromise
   * @param step       stride through the path; 1 counts every pose
   */
  static double evaluate(const std::vector<std::vector<double>>& path,
                         const std::vector<std::vector<double>>& footprint,
                         double metres_per_unit, int supersample = 4,
                         int step = 1) {
    if (path.empty() || footprint.size() < 3) return 0.0;
    if (supersample < 1) supersample = 1;
    if (step < 1) step = 1;

    // Bounding box of every footprint placed along the path, so that a robot
    // overhanging the map boundary is still counted.
    double reach = 0.0;
    for (const auto& p : footprint)
      reach = std::max(reach, std::hypot(p[0], p[1]));

    double minx = path[0][0], maxx = path[0][0];
    double miny = path[0][1], maxy = path[0][1];
    for (const auto& q : path) {
      minx = std::min(minx, q[0]); maxx = std::max(maxx, q[0]);
      miny = std::min(miny, q[1]); maxy = std::max(maxy, q[1]);
    }
    minx -= reach; maxx += reach; miny -= reach; maxy += reach;

    const int W = static_cast<int>(std::ceil((maxx - minx) * supersample)) + 2;
    const int H = static_cast<int>(std::ceil((maxy - miny) * supersample)) + 2;
    if (W <= 0 || H <= 0) return 0.0;
    std::vector<bool> covered(static_cast<size_t>(W) * H, false);

    std::vector<double> px(footprint.size()), py(footprint.size());
    for (size_t i = 0; i < path.size(); i += static_cast<size_t>(step)) {
      const double c = std::cos(path[i][2]), s = std::sin(path[i][2]);
      for (size_t k = 0; k < footprint.size(); ++k) {
        const double fx = footprint[k][0], fy = footprint[k][1];
        px[k] = ((fx * c - fy * s) + path[i][0] - minx) * supersample;
        py[k] = ((fx * s + fy * c) + path[i][1] - miny) * supersample;
      }
      fillPolygon(px, py, W, H, covered);
    }

    size_t cells = 0;
    for (bool b : covered) cells += b ? 1 : 0;
    const double cell = metres_per_unit / supersample;
    return static_cast<double>(cells) * cell * cell;
  }

 private:
  /// Scanline fill of a convex or concave simple polygon into the raster.
  static void fillPolygon(const std::vector<double>& px,
                          const std::vector<double>& py, int W, int H,
                          std::vector<bool>& covered) {
    const size_t n = px.size();
    double ymin = py[0], ymax = py[0];
    for (size_t i = 1; i < n; ++i) {
      ymin = std::min(ymin, py[i]);
      ymax = std::max(ymax, py[i]);
    }
    int y0 = std::max(0, static_cast<int>(std::floor(ymin)));
    int y1 = std::min(H - 1, static_cast<int>(std::ceil(ymax)));

    std::vector<double> xs;
    for (int y = y0; y <= y1; ++y) {
      const double yc = y + 0.5;
      xs.clear();
      for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double yi = py[i], yj = py[j];
        if ((yi <= yc && yj > yc) || (yj <= yc && yi > yc)) {
          const double t = (yc - yi) / (yj - yi);
          xs.push_back(px[i] + t * (px[j] - px[i]));
        }
      }
      if (xs.size() < 2) continue;
      std::sort(xs.begin(), xs.end());
      for (size_t k = 0; k + 1 < xs.size(); k += 2) {
        // Boundary-inclusive: a cell the polygon merely touches is counted.
        // Sampling at cell centres instead would systematically under-report a
        // shape as thin as a forklift, whose width spans only a few cells.
        int xa = std::max(0, static_cast<int>(std::floor(xs[k])));
        int xb = std::min(W - 1, static_cast<int>(std::ceil(xs[k + 1])) - 1);
        for (int x = xa; x <= xb; ++x)
          covered[static_cast<size_t>(y) * W + x] = true;
      }
    }
  }
};
