// Offline unit test for OccupancyGrid + PathPlanner (no DDS, no simulator).
//  1. a wall seen by synthetic scans blocks the straight line and A* goes around it;
//  2. a robot starting INSIDE the inflation halo of a thin wall escapes on its own
//     side and never plans through the wall;
//  3. a low box (below the horizontal layer) is not erased by rays passing above it.
#include <cmath>
#include <cstdio>

#include "go2_autonomy/perception/occupancy_grid.hpp"
#include "go2_autonomy/planning/path_planner.hpp"

using namespace go2;

static int failures = 0;
static void check(bool ok, const char* what) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

// Synthetic body-frame scan of an axis-aligned wall segment x = wx, |y - wy| < hy, height 0.6 m,
// seen from a robot at (rx, ry) facing +x with the sensor 0.42 m above ground.
static PointCloud wallScan(double rx, double ry, double wx, double wy, double hy) {
  PointCloud pc;
  for (double y = wy - hy; y <= wy + hy; y += 0.02)
    for (double z : {0.1, 0.3, 0.5}) pc.points.push_back({wx - rx, y - ry, z - 0.32});
  pc.seq = 1;
  return pc;
}

static bool pathCrossesOccupied(const OccupancyGrid& g, const Path& p) {
  for (size_t i = 1; i < p.waypoints.size(); ++i) {
    const auto& a = p.waypoints[i - 1];
    const auto& b = p.waypoints[i];
    const int n = (int)std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.01);
    for (int k = 0; k <= n; ++k) {
      int cx, cy;
      if (g.worldToCell(a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n, cx, cy) && g.occupied(cx, cy))
        return true;
    }
  }
  return false;
}

int main() {
  PathPlanner planner;
  {
    OccupancyGrid g;
    for (int i = 0; i < 3; ++i) g.integrate(wallScan(0, 0, 2.5, 0, 0.6), {0, 0, 0}, 0, 0, 0.32);
    g.updateCostmap();
    const auto cost = g.costSnapshot();
    check(!PathPlanner::segmentFree(g, cost, 0, 0, 5, 0, 254), "wall blocks the straight line");
    const Path p = planner.plan(g, cost, 0, 0, 5, 0);
    check(p.valid && !pathCrossesOccupied(g, p) && p.length > 5.0, "A* detours around the wall");
  }
  {
    OccupancyGrid g;
    // thin wall at x = 1.0, robot 0.2 m in front of it (inside the 0.34 m lethal halo)
    for (int i = 0; i < 3; ++i) g.integrate(wallScan(0.8, 0, 1.0, 0, 1.5), {0.8, 0, 0}, 0, 0, 0.32);
    g.updateCostmap();
    const auto cost = g.costSnapshot();
    check(g.costAtWorld(0.8, 0) >= 254, "start is inside the inflation halo");
    const Path p = planner.plan(g, cost, 0.8, 0, 1.6, 0);  // goal just behind the wall
    check(p.valid, "escape + plan succeeds");
    check(!pathCrossesOccupied(g, p), "planned path never crosses an occupied cell");
    check(p.valid && p.waypoints.size() > 2 && p.waypoints[1].x < 1.0, "escape waypoint is on the robot's side");
  }
  {
    OccupancyGrid g;
    // low box top at 0.25 m around x = 1.5 seen by a tilted beam, plus horizontal rays passing over it
    PointCloud box;
    for (double y = -0.3; y <= 0.3; y += 0.03) box.points.push_back({1.5, y, 0.25 - 0.32 - 0.1});
    PointCloud over;
    for (double y = -0.3; y <= 0.3; y += 0.03) over.points.push_back({4.0, y, -0.32});  // ground hits behind the box
    for (int i = 0; i < 5; ++i) {
      g.integrate(box, {0, 0, 0}, 0, 0, 0.32);
      g.integrate(over, {0, 0, 0}, 0, 0, 0.32);
    }
    int cx, cy;
    g.worldToCell(1.5, 0.0, cx, cy);
    check(g.occupied(cx, cy), "low box survives rays passing above it");
  }
  std::printf("%s (%d failures)\n", failures ? "PLANNER TESTS FAILED" : "PLANNER TESTS PASSED", failures);
  return failures ? 1 : 0;
}
