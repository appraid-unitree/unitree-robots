#include "go2_autonomy/planning/path_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

namespace go2 {

bool PathPlanner::segmentFree(const OccupancyGrid& grid, const std::vector<uint8_t>& cost, double ax, double ay,
                              double bx, double by, int threshold) {
  const double res = grid.config().resolution;
  const double len = std::hypot(bx - ax, by - ay);
  const int n = std::max(1, (int)std::ceil(len / (0.5 * res)));
  const int W = grid.config().width;
  for (int i = 0; i <= n; ++i) {
    const double t = (double)i / n;
    int cx, cy;
    if (!grid.worldToCell(ax + t * (bx - ax), ay + t * (by - ay), cx, cy)) return false;
    if (cost[cy * W + cx] >= threshold) return false;
  }
  return true;
}

Path PathPlanner::plan(const OccupancyGrid& grid, const std::vector<uint8_t>& cost, double sx, double sy, double gx,
                       double gy) const {
  Path out;
  const auto& gc = grid.config();
  const int W = gc.width, H = gc.height;
  int scx, scy, gcx, gcy;
  if (!grid.worldToCell(sx, sy, scx, scy) || !grid.worldToCell(gx, gy, gcx, gcy)) return out;
  int start = scy * W + scx;
  const int goal = gcy * W + gcx;
  if (cost[goal] >= cfg_.lethal_threshold) return out;  // goal inside an obstacle

  // Robot already inside an obstacle's inflation halo (tracking error, obstacle
  // seen late): first escape to the NEAREST non-lethal cell by BFS that never
  // enters an occupied cell, then plan normally from there. The planner itself
  // never traverses lethal cells, so a path can never cut through an obstacle.
  bool escaped = false;
  double esc_x = sx, esc_y = sy;
  if (cost[start] >= cfg_.lethal_threshold) {
    std::vector<int> prev((size_t)W * H, -2);
    std::vector<int> q{start};
    prev[start] = -1;
    const int max_steps = (int)std::ceil(0.8 / gc.resolution);
    int found = -1;
    for (size_t qi = 0; qi < q.size() && found < 0; ++qi) {
      const int c = q[qi], cx = c % W, cy = c / W;
      if (std::abs(cx - scx) > max_steps || std::abs(cy - scy) > max_steps) continue;
      static const int ex[8] = {1, -1, 0, 0, 1, 1, -1, -1}, ey[8] = {0, 0, 1, -1, 1, -1, 1, -1};
      for (int k = 0; k < 8; ++k) {
        const int x = cx + ex[k], y = cy + ey[k];
        if (x < 0 || y < 0 || x >= W || y >= H) continue;
        const int j = y * W + x;
        if (prev[j] != -2) continue;
        if (grid.clearanceCell(x, y) < 0.5 * gc.resolution) continue;  // occupied: never pass through
        prev[j] = c;
        if (cost[j] < cfg_.lethal_threshold) { found = j; break; }
        q.push_back(j);
      }
    }
    if (found < 0) return out;
    start = found;
    escaped = true;
    grid.cellToWorld(found % W, found / W, esc_x, esc_y);
  }

  const auto t0 = std::chrono::steady_clock::now();
  std::vector<float> g((size_t)W * H, std::numeric_limits<float>::max());
  std::vector<int> parent((size_t)W * H, -1);
  std::vector<uint8_t> closed((size_t)W * H, 0);
  using Item = std::pair<float, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  auto h = [&](int i) {
    const int x = i % W, y = i / W;
    const double dx = std::abs(x - gcx), dy = std::abs(y - gcy);
    return (float)(gc.resolution * ((dx + dy) + (1.41421356 - 2.0) * std::min(dx, dy)));
  };
  g[start] = 0.0f;
  open.push({h(start), start});
  const int nx[8] = {1, -1, 0, 0, 1, 1, -1, -1}, ny[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  bool found = false;
  int expansions = 0;
  while (!open.empty()) {
    const int cur = open.top().second;
    open.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    if (cur == goal) { found = true; break; }
    if ((++expansions & 1023) == 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > cfg_.max_plan_time_s)
      break;
    const int cx = cur % W, cy = cur / W;
    for (int k = 0; k < 8; ++k) {
      const int x = cx + nx[k], y = cy + ny[k];
      if (x < 0 || y < 0 || x >= W || y >= H) continue;
      const int j = y * W + x;
      if (closed[j]) continue;
      const uint8_t c = cost[j];
      if (c >= cfg_.lethal_threshold) continue;  // never through an inflated obstacle
      const double step = gc.resolution * (k < 4 ? 1.0 : 1.41421356);
      const double penalty = cfg_.cost_weight * (c / 253.0);
      const float ng = g[cur] + (float)(step * (1.0 + penalty));
      if (ng < g[j]) {
        g[j] = ng;
        parent[j] = cur;
        open.push({ng + h(j), j});
      }
    }
  }
  if (!found) return out;

  std::vector<int> cells;
  for (int c = goal; c != -1; c = parent[c]) cells.push_back(c);
  std::reverse(cells.begin(), cells.end());
  std::vector<Vec3> raw;
  raw.reserve(cells.size());
  for (int c : cells) {
    double x, y;
    grid.cellToWorld(c % W, c / W, x, y);
    raw.push_back({x, y, 0});
  }
  if (escaped) {
    raw.front() = {esc_x, esc_y, 0};
    raw.insert(raw.begin(), Vec3{sx, sy, 0});
  } else {
    raw.front() = {sx, sy, 0};
  }
  if (raw.size() < 2) raw.push_back({gx, gy, 0});
  raw.back() = {gx, gy, 0};

  // Line-of-sight shortcutting: keep a waypoint only when the direct segment
  // would enter higher-cost cells than the A* path itself tolerated.
  std::vector<Vec3> smooth{raw.front()};
  size_t i = 0;
  if (escaped) { smooth.push_back(raw[1]); i = 1; }  // keep the explicit escape segment
  while (i + 1 < raw.size()) {
    size_t best = i + 1;
    for (size_t j = raw.size() - 1; j > i + 1; --j) {
      if (segmentFree(grid, cost, raw[i].x, raw[i].y, raw[j].x, raw[j].y, 100)) { best = j; break; }
    }
    smooth.push_back(raw[best]);
    i = best;
  }
  out.waypoints = smooth;
  for (size_t k = 1; k < smooth.size(); ++k)
    out.length += std::hypot(smooth[k].x - smooth[k - 1].x, smooth[k].y - smooth[k - 1].y);
  out.valid = true;
  return out;
}

}  // namespace go2
