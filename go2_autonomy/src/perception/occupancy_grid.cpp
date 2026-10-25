#include "go2_autonomy/perception/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace go2 {

OccupancyGrid::OccupancyGrid(const GridConfig& cfg) : cfg_(cfg) { clear(); }

void OccupancyGrid::clear() {
  std::lock_guard<std::mutex> l(mtx);
  logodds_.assign((size_t)cfg_.width * cfg_.height, 0.0f);
  cost_.assign((size_t)cfg_.width * cfg_.height, 0);
  dist_.assign((size_t)cfg_.width * cfg_.height, (float)cfg_.soft_radius);
}

bool OccupancyGrid::worldToCell(double x, double y, int& cx, int& cy) const {
  cx = (int)std::floor((x - cfg_.origin_x) / cfg_.resolution);
  cy = (int)std::floor((y - cfg_.origin_y) / cfg_.resolution);
  return inBounds(cx, cy);
}

void OccupancyGrid::cellToWorld(int cx, int cy, double& x, double& y) const {
  x = cfg_.origin_x + (cx + 0.5) * cfg_.resolution;
  y = cfg_.origin_y + (cy + 0.5) * cfg_.resolution;
}

bool OccupancyGrid::occupied(int cx, int cy) const {
  return inBounds(cx, cy) && logodds_[idx(cx, cy)] > cfg_.l_occ;
}

uint8_t OccupancyGrid::cost(int cx, int cy) const {
  if (!inBounds(cx, cy)) return 255;
  return cost_[idx(cx, cy)];
}

uint8_t OccupancyGrid::costAtWorld(double x, double y) const {
  int cx, cy;
  if (!worldToCell(x, y, cx, cy)) return 255;
  return cost_[idx(cx, cy)];
}

double OccupancyGrid::clearance(double x, double y) const {
  int cx, cy;
  if (!worldToCell(x, y, cx, cy)) return 0.0;
  return dist_[idx(cx, cy)];
}

void OccupancyGrid::rayClear(int x0, int y0, int x1, int y1, double z0, double z1) {
  // Bresenham. A cell is evidence of free space only where the ray passes it
  // BELOW clear_below_z: anything taller would have intercepted the ray.
  // (A ray passing high over a low box says nothing about the box.)
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  const double total = std::hypot((double)(x1 - x0), (double)(y1 - y0));
  while (!(x == x1 && y == y1)) {
    const double t = total > 0 ? std::hypot((double)(x - x0), (double)(y - y0)) / total : 1.0;
    const double alt = z0 + t * (z1 - z0);
    if (alt < cfg_.clear_below_z && inBounds(x, y)) {
      float& l = logodds_[idx(x, y)];
      l = std::max((float)cfg_.l_min, l + (float)cfg_.l_miss);
    }
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
}

ScanStats OccupancyGrid::integrate(const PointCloud& cloud, const Pose2D& pose, double roll, double pitch,
                                   double base_height) {
  ScanStats st;
  // body -> levelled frame (roll, pitch), then yaw.
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
  const double cyaw = std::cos(pose.yaw), syaw = std::sin(pose.yaw);
  auto toLevel = [&](const Vec3& p) {
    // R = Ry(pitch) * Rx(roll)
    const double y1 = cr * p.y - sr * p.z, z1 = sr * p.y + cr * p.z;
    return Vec3{cp * p.x + sp * z1, y1, -sp * p.x + cp * z1};
  };
  const Vec3 sensor_l = toLevel({0, 0, cfg_.sensor_height_offset});
  const double sx = pose.x + cyaw * sensor_l.x - syaw * sensor_l.y;
  const double sy = pose.y + syaw * sensor_l.x + cyaw * sensor_l.y;
  const double sensor_z = base_height + sensor_l.z;
  int scx, scy;
  const bool sensor_in = worldToCell(sx, sy, scx, scy);

  std::lock_guard<std::mutex> l(mtx);
  for (const Vec3& pb : cloud.points) {
    if (std::fabs(pb.x) < cfg_.self_filter_x && std::fabs(pb.y) < cfg_.self_filter_y) { st.ignored_points++; continue; }
    const Vec3 pl = toLevel(pb);
    const double z = base_height + pl.z;  // height above ground
    const double r = std::hypot(pl.x, pl.y);
    if (r > cfg_.max_range) { st.ignored_points++; continue; }
    const double wx = pose.x + cyaw * pl.x - syaw * pl.y;
    const double wy = pose.y + syaw * pl.x + cyaw * pl.y;
    int cx, cy;
    const bool in = worldToCell(wx, wy, cx, cy);
    if (z > cfg_.obstacle_max_z) { st.ignored_points++; continue; }
    if (sensor_in && in && z <= cfg_.ground_max_z) rayClear(scx, scy, cx, cy, sensor_z, z);
    if (!in) continue;
    float& lo = logodds_[idx(cx, cy)];
    if (z >= cfg_.obstacle_min_z) {
      lo = std::min((float)cfg_.l_max, lo + (float)cfg_.l_hit);
      st.obstacle_points++;
    } else if (z <= cfg_.ground_max_z) {
      lo = std::max((float)cfg_.l_min, lo + (float)cfg_.l_miss);
      st.ground_points++;
    } else {
      st.ignored_points++;
    }
  }
  return st;
}

void OccupancyGrid::updateCostmap() {
  std::lock_guard<std::mutex> l(mtx);
  const int W = cfg_.width, H = cfg_.height;
  const float big = (float)cfg_.soft_radius;
  // Brushfire (multi-source Dijkstra, 8-connected) distance transform, capped.
  std::vector<float> d((size_t)W * H, std::numeric_limits<float>::max());
  using Item = std::pair<float, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  for (int i = 0; i < W * H; ++i)
    if (logodds_[i] > cfg_.l_occ) { d[i] = 0.0f; pq.push({0.0f, i}); }
  const float res = (float)cfg_.resolution;
  const int nx[8] = {1, -1, 0, 0, 1, 1, -1, -1}, ny[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const float nd[8] = {res, res, res, res, res * 1.41421356f, res * 1.41421356f, res * 1.41421356f, res * 1.41421356f};
  while (!pq.empty()) {
    auto [dist, i] = pq.top();
    pq.pop();
    if (dist > d[i] || dist > big) continue;
    const int cx = i % W, cy = i / W;
    for (int k = 0; k < 8; ++k) {
      const int x = cx + nx[k], y = cy + ny[k];
      if (x < 0 || y < 0 || x >= W || y >= H) continue;
      const int j = y * W + x;
      const float nd2 = dist + nd[k];
      if (nd2 < d[j]) { d[j] = nd2; pq.push({nd2, j}); }
    }
  }
  for (int i = 0; i < W * H; ++i) {
    const float di = std::min(d[i], big);
    dist_[i] = di;
    if (di <= cfg_.lethal_radius) cost_[i] = 254;
    else {
      const double f = (cfg_.soft_radius - di) / (cfg_.soft_radius - cfg_.lethal_radius);
      cost_[i] = (uint8_t)clamp(std::round(253.0 * f * f), 0.0, 253.0);
    }
  }
}

std::vector<uint8_t> OccupancyGrid::costSnapshot() const {
  std::lock_guard<std::mutex> l(mtx);
  return cost_;
}

std::vector<uint8_t> OccupancyGrid::occupancySnapshot() const {
  std::lock_guard<std::mutex> l(mtx);
  std::vector<uint8_t> out(logodds_.size());
  for (size_t i = 0; i < logodds_.size(); ++i)
    out[i] = logodds_[i] > cfg_.l_occ ? 2 : (logodds_[i] < -0.3f ? 1 : 0);
  return out;
}

int OccupancyGrid::occupiedCount() const {
  std::lock_guard<std::mutex> l(mtx);
  int n = 0;
  for (float v : logodds_) n += v > cfg_.l_occ;
  return n;
}

}  // namespace go2
