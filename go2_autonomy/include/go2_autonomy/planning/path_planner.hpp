// Grid A* global planner on the inflated cost map + line-of-sight smoothing.
#pragma once

#include <cstdint>
#include <vector>

#include "go2_autonomy/common/math.hpp"
#include "go2_autonomy/perception/occupancy_grid.hpp"

namespace go2 {

struct PlannerConfig {
  double cost_weight = 3.0;      // extra path length per unit soft cost (cost/253)
  int lethal_threshold = 254;    // cells >= this are blocked
  double max_plan_time_s = 0.5;
  bool allow_unknown = true;     // unknown cells are treated as free (optimistic)
};

struct Path {
  std::vector<Vec3> waypoints;  // x, y (z unused)
  double length = 0.0;
  bool valid = false;
};

class PathPlanner {
 public:
  explicit PathPlanner(const PlannerConfig& cfg = PlannerConfig()) : cfg_(cfg) {}

  // Plans from start to goal on a snapshot of the cost layer.
  // If the start cell is inside the inflated region (robot already close to
  // an obstacle) it escapes to the nearest non-lethal cell first.
  Path plan(const OccupancyGrid& grid, const std::vector<uint8_t>& cost, double sx, double sy, double gx,
            double gy) const;

  // True if the straight segment a->b only crosses cells with cost < threshold.
  static bool segmentFree(const OccupancyGrid& grid, const std::vector<uint8_t>& cost, double ax, double ay,
                          double bx, double by, int threshold);

 private:
  PlannerConfig cfg_;
};

}  // namespace go2
