// 2-D log-odds occupancy grid built from range points.
//
// Points arrive in the body frame. They are transformed with the estimated
// planar pose and IMU roll/pitch, classified by their height above the ground
// (ground / obstacle / overhead), and fused into the grid. Free space is
// cleared along each ray. An inflated "cost" layer is derived for planning.
#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include "go2_autonomy/common/math.hpp"
#include "go2_autonomy/estimation/state_estimator.hpp"
#include "go2_autonomy/perception/point_cloud.hpp"

namespace go2 {

struct GridConfig {
  double resolution = 0.05;                 // [m]
  double origin_x = -6.0, origin_y = -6.0;  // world coords of cell (0,0) corner
  int width = 280, height = 240;            // cells (14 m x 12 m)
  double sensor_height_offset = 0.10;       // scanner mount above base origin (matches simulator)
  double obstacle_min_z = 0.07;             // above ground -> obstacle
  double obstacle_max_z = 0.85;             // higher points (overhangs) ignored
  double ground_max_z = 0.04;               // below -> ground (free) evidence
  double clear_below_z = 0.12;              // ray altitude below which traversed cells are cleared
  double self_filter_x = 0.42, self_filter_y = 0.24;  // body-frame box ignored (own legs)
  double max_range = 5.0;
  double l_hit = 0.85, l_miss = -0.4, l_min = -2.0, l_max = 3.5, l_occ = 0.6;
  // Planning footprint.
  double lethal_radius = 0.34;   // robot circumscribed half-width + margin [m]
  double soft_radius = 0.70;     // cost decays to zero at this distance [m]
};

struct ScanStats {
  int obstacle_points = 0, ground_points = 0, ignored_points = 0;
};

class OccupancyGrid {
 public:
  explicit OccupancyGrid(const GridConfig& cfg = GridConfig());

  const GridConfig& config() const { return cfg_; }
  void clear();

  // base_height: estimated height of the base origin above the ground.
  ScanStats integrate(const PointCloud& cloud, const Pose2D& pose, double roll, double pitch, double base_height);

  bool worldToCell(double x, double y, int& cx, int& cy) const;
  void cellToWorld(int cx, int cy, double& x, double& y) const;
  bool inBounds(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < cfg_.width && cy < cfg_.height; }

  bool occupied(int cx, int cy) const;
  // 0 = free, 1..253 = soft cost, 254 = lethal (inside inflated obstacle), 255 = outside map.
  uint8_t cost(int cx, int cy) const;
  uint8_t costAtWorld(double x, double y) const;
  // Distance [m] from world point to the nearest occupied cell (capped at soft_radius).
  double clearance(double x, double y) const;
  double clearanceCell(int cx, int cy) const { return inBounds(cx, cy) ? dist_[idx(cx, cy)] : 0.0; }

  // Recompute the inflated cost layer (call after integrate()).
  void updateCostmap();

  // Thread-safe snapshot of the cost layer for the planner.
  std::vector<uint8_t> costSnapshot() const;
  int occupiedCount() const;
  // 0 = unknown, 1 = free, 2 = occupied (thread-safe copy, for logging / visualisation).
  std::vector<uint8_t> occupancySnapshot() const;

  mutable std::mutex mtx;  // guards logodds_ / cost_ / dist_ for cross-thread use

 private:
  int idx(int cx, int cy) const { return cy * cfg_.width + cx; }
  void rayClear(int x0, int y0, int x1, int y1, double z0, double z1);

  GridConfig cfg_;
  std::vector<float> logodds_;
  std::vector<uint8_t> cost_;
  std::vector<float> dist_;
};

}  // namespace go2
