// go2_acceptance_tests -- end-to-end acceptance tests against unitree_mujoco_nav
// (scene_nav.xml) over real DDS command/feedback.
//
// Every verdict is computed from MEASURED feedback:
//   * rt/lowstate (IMU, joint encoders)        -> attitude, yaw, stability
//   * rt/sportmodestate (simulator truth pose) -> distance, drift, goal error
//   * an independent collision oracle that checks the measured trajectory
//     against the known obstacle geometry of scene_nav.xml (test-side only;
//     the navigation stack itself never sees this geometry).
//
// usage: go2_acceptance_tests [--iface lo] [--domain 1] [--pose sport|odom] [--log out_dir]
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "go2_autonomy/app/go2_robot.hpp"
#include "go2_autonomy/navigation/navigator.hpp"

using namespace go2;
using clk = std::chrono::steady_clock;

// ------------------------------------------------------------ scene oracle
struct BoxObs { const char* name; double cx, cy, hx, hy; };
struct CylObs { const char* name; double cx, cy, r; };
static const BoxObs kBoxes[] = {{"obstacle_wall", 2.5, 0.0, 0.2, 0.6}, {"obstacle_low_box", 3.6, -1.0, 0.3, 0.3}};
static const CylObs kCyls[] = {{"obstacle_pillar", 4.0, 1.1, 0.2}};
// Robot collision footprint (trunk + head + feet), base frame.
constexpr double kFootHalfLen = 0.34, kFootHalfWid = 0.17;

static double pointObstacleDistance(double x, double y) {
  double d = 1e9;
  for (const auto& b : kBoxes) {
    const double dx = std::max(std::fabs(x - b.cx) - b.hx, 0.0), dy = std::max(std::fabs(y - b.cy) - b.hy, 0.0);
    const bool inside = std::fabs(x - b.cx) <= b.hx && std::fabs(y - b.cy) <= b.hy;
    d = std::min(d, inside ? -std::min(b.hx - std::fabs(x - b.cx), b.hy - std::fabs(y - b.cy)) : std::hypot(dx, dy));
  }
  for (const auto& c : kCyls) d = std::min(d, std::hypot(x - c.cx, y - c.cy) - c.r);
  return d;
}

// Minimum signed distance between the robot footprint rectangle and obstacles.
static double footprintClearance(double x, double y, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  double d = 1e9;
  const int N = 12;
  for (int i = 0; i <= N; ++i) {
    const double u = -kFootHalfLen + 2.0 * kFootHalfLen * i / N;
    const double v = -kFootHalfWid + 2.0 * kFootHalfWid * i / N;
    const double pts[4][2] = {{u, kFootHalfWid}, {u, -kFootHalfWid}, {kFootHalfLen, v}, {-kFootHalfLen, v}};
    for (auto& p : pts) d = std::min(d, pointObstacleDistance(x + c * p[0] - s * p[1], y + s * p[0] + c * p[1]));
  }
  return d;
}

// ------------------------------------------------------------ measurement
struct Truth {
  double x = 0, y = 0, z = 0, yaw = 0, roll = 0, pitch = 0;
  bool valid = false;
};

class Recorder {
 public:
  Recorder(Go2Robot& r, Navigator& n, const std::string& path) : robot_(r), nav_(n) {
    if (!path.empty()) {
      f_.open(path);
      f_ << "t,test,gt_x,gt_y,gt_z,gt_yaw,roll,pitch,odo_x,odo_y,odo_yaw,nav_x,nav_y,nav_yaw,cmd_vx,cmd_wz,"
            "loco_mode,nav_status,clearance,qw,qx,qy,qz,q0,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11\n";
    }
    t0_ = clk::now();
    th_ = std::thread([this] { loop(); });
  }
  ~Recorder() { run_ = false; th_.join(); }

  Truth truth() const {
    Truth t;
    const ExternalPose p = robot_.io().sportState();
    const LowLevelState s = robot_.lastState();
    const Mat3 R = quatToMat(s.imu_quat[0], s.imu_quat[1], s.imu_quat[2], s.imu_quat[3]);
    t.x = p.x; t.y = p.y; t.z = p.z;
    t.yaw = std::atan2(R(1, 0), R(0, 0));
    t.roll = s.imu_rpy[0]; t.pitch = s.imu_rpy[1];
    t.valid = p.valid && s.valid;
    return t;
  }
  void setTest(const std::string& name) {
    std::lock_guard<std::mutex> l(m_);
    test_ = name;
    min_clear_ = 1e9; max_tilt_ = 0; yaw_unwrapped_ = 0; have_prev_ = false;
  }
  double minClearance() const { std::lock_guard<std::mutex> l(m_); return min_clear_; }
  double maxTilt() const { std::lock_guard<std::mutex> l(m_); return max_tilt_; }
  double yawTravelled() const { std::lock_guard<std::mutex> l(m_); return yaw_unwrapped_; }
  double lateralMax(double ox, double oy, double oyaw) const {
    std::lock_guard<std::mutex> l(m_);
    double m = 0;
    for (auto& p : traj_) m = std::max(m, std::fabs(-std::sin(oyaw) * (p.first - ox) + std::cos(oyaw) * (p.second - oy)));
    return m;
  }
  void clearTraj() { std::lock_guard<std::mutex> l(m_); traj_.clear(); }

 private:
  void loop() {
    while (run_) {
      const Truth t = truth();
      if (t.valid) {
        const double clear = footprintClearance(t.x, t.y, t.yaw);
        const EstimatedState odo = robot_.estimator().legOdometryEstimate();
        const EstimatedState navp = robot_.estimate();
        const NavTelemetry nt = nav_.telemetry();
        std::lock_guard<std::mutex> l(m_);
        min_clear_ = std::min(min_clear_, clear);
        max_tilt_ = std::max(max_tilt_, std::max(std::fabs(t.roll), std::fabs(t.pitch)));
        if (have_prev_) yaw_unwrapped_ += wrapAngle(t.yaw - prev_yaw_);
        prev_yaw_ = t.yaw; have_prev_ = true;
        traj_.push_back({t.x, t.y});
        if (f_.is_open()) {
          const double tt = std::chrono::duration<double>(clk::now() - t0_).count();
          char buf[512];
          std::snprintf(buf, sizeof(buf),
                        "%.3f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%s,%s,%.4f\n", tt,
                        test_.c_str(), t.x, t.y, t.z, t.yaw, t.roll, t.pitch, odo.pose.x, odo.pose.y, odo.pose.yaw,
                        navp.pose.x, navp.pose.y, navp.pose.yaw, nt.cmd.vx, nt.cmd.wz,
                        toString(robot_.locomotion().mode()), toString(nt.status), clear);
          f_ << buf;
          // replay data: IMU quaternion + 12 joint positions (rt/lowstate)
          const LowLevelState ls = robot_.lastState();
          std::string extra;
          for (int k = 0; k < 4; ++k) { std::snprintf(buf, sizeof(buf), ",%.5f", ls.imu_quat[k]); extra += buf; }
          for (int k = 0; k < kNumMotors; ++k) { std::snprintf(buf, sizeof(buf), ",%.4f", ls.q[k]); extra += buf; }
          f_.seekp(-1, std::ios_base::cur);  // overwrite the newline
          f_ << extra << "\n";
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  Go2Robot& robot_;
  Navigator& nav_;
  std::ofstream f_;
  std::thread th_;
  std::atomic<bool> run_{true};
  mutable std::mutex m_;
  std::string test_ = "init";
  double min_clear_ = 1e9, max_tilt_ = 0, yaw_unwrapped_ = 0, prev_yaw_ = 0;
  bool have_prev_ = false;
  std::vector<std::pair<double, double>> traj_;
  clk::time_point t0_;
};

// ------------------------------------------------------------ test bookkeeping
struct TestResult { int id; std::string name; bool pass; std::string detail; };
static std::vector<TestResult> g_results;

static void report(int id, const std::string& name, bool pass, const std::string& detail) {
  g_results.push_back({id, name, pass, detail});
  std::printf("\n==== TEST %d  %-42s %s\n     %s\n\n", id, name.c_str(), pass ? "PASS" : "FAIL", detail.c_str());
  std::fflush(stdout);
}

static std::string f(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return buf;
}

static void sleepS(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

static bool waitMode(Go2Robot& r, LocoMode m, double timeout) {
  const auto t0 = clk::now();
  while (std::chrono::duration<double>(clk::now() - t0).count() < timeout) {
    if (r.locomotion().mode() == m) return true;
    sleepS(0.02);
  }
  return false;
}

// Open-loop locomotion primitive: walk with a constant body twist, then stop.
static void walkFor(Go2Robot& r, const Twist2D& cmd, double seconds) {
  r.setVelocity(cmd);
  r.setWalking(true);
  sleepS(seconds);
  r.setVelocity({});
  r.setWalking(false);
  waitMode(r, LocoMode::STAND, 5.0);
  sleepS(0.5);
}

static void dumpMap(const Navigator& nav, const std::string& path) {
  if (path.empty()) return;
  const auto occ = nav.grid().occupancySnapshot();
  const auto& gc = nav.grid().config();
  std::ofstream o(path);
  o << "# origin_x " << gc.origin_x << " origin_y " << gc.origin_y << " res " << gc.resolution << " width "
    << gc.width << " height " << gc.height << "\n";
  for (int y = 0; y < gc.height; ++y) {
    for (int x = 0; x < gc.width; ++x) o << (int)occ[y * gc.width + x] << (x + 1 < gc.width ? "," : "");
    o << "\n";
  }
}

static void dumpPath(const Path& p, const std::string& path) {
  if (path.empty()) return;
  std::ofstream o(path);
  o << "x,y\n";
  for (auto& w : p.waypoints) o << w.x << "," << w.y << "\n";
}

int main(int argc, char** argv) {
  RobotConfig rc;
  std::string log_dir;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--iface" && i + 1 < argc) rc.dds.network_interface = argv[++i];
    else if (a == "--domain" && i + 1 < argc) rc.dds.domain_id = std::atoi(argv[++i]);
    else if (a == "--pose" && i + 1 < argc) rc.pose_source = std::string(argv[++i]) == "odom" ? PoseSource::LEG_ODOMETRY : PoseSource::SPORT_STATE;
    else if (a == "--log" && i + 1 < argc) log_dir = argv[++i];
  }
  if (rc.dds.domain_id == 0) {
    std::cerr << "go2_acceptance_tests is simulation-only (scene_nav.xml oracle); refusing DDS domain 0." << std::endl;
    return 2;
  }
  auto L = [&](const std::string& n) { return log_dir.empty() ? std::string() : log_dir + "/" + n; };

  Go2Robot robot(rc);
  if (!robot.start()) { std::cerr << "no rt/lowstate -- is unitree_mujoco_nav running?" << std::endl; return 1; }
  NavConfig nc;
  Navigator nav(robot, nc);
  nav.setLogger([](const std::string& s) { std::cout << s << std::endl; });
  Recorder rec(robot, nav, L("trajectory.csv"));
  sleepS(0.5);
  const auto t_start = clk::now();
  const uint32_t tick_start = robot.lastState().tick;

  // ---------------- 1. stand -------------------------------------------------
  {
    rec.setTest("1_stand");
    const bool up = robot.standUp(20.0);
    sleepS(0.5);
    const Truth a = rec.truth();
    rec.setTest("1_stand");  // reset tilt stats: measure the HOLD phase only
    sleepS(3.0);
    const Truth b = rec.truth();
    const double drift = std::hypot(b.x - a.x, b.y - a.y);
    const bool pass = up && b.z > 0.27 && b.z < 0.38 && rec.maxTilt() < 0.08 && drift < 0.03;
    report(1, "Robot stands stably", pass,
           f("stand-up %s; IMU-site height %.3f m (base origin ~0.04 m lower); max |roll|,|pitch| over 3 s hold %.3f rad; horizontal drift %.3f m",
             up ? "completed (encoder-settled)" : "FAILED", b.z, rec.maxTilt(), drift));
    if (!up) return 1;
  }
  nav.start();  // mapping from now on
  sleepS(0.5);

  // ---------------- 2. straight walking (forward + backward) -----------------
  {
    rec.setTest("2_walk_straight");
    const Truth a = rec.truth();
    walkFor(robot, {0.4, 0, 0}, 4.0);
    const Truth b = rec.truth();
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double fwd = std::cos(a.yaw) * dx + std::sin(a.yaw) * dy;
    const double lat = -std::sin(a.yaw) * dx + std::cos(a.yaw) * dy;
    const double dyaw = wrapAngle(b.yaw - a.yaw);
    rec.setTest("2b_walk_backward");
    const Truth c = rec.truth();
    walkFor(robot, {-0.3, 0, 0}, 3.0);
    const Truth d = rec.truth();
    const double back = std::cos(c.yaw) * (d.x - c.x) + std::sin(c.yaw) * (d.y - c.y);
    const bool pass = fwd > 1.2 && std::fabs(lat) < 0.2 && std::fabs(dyaw) < 0.15 && back < -0.6 && rec.maxTilt() < 0.2;
    report(2, "Robot walks forward in a straight line", pass,
           f("cmd vx=0.40 m/s for 4 s: forward %.2f m (avg %.2f m/s), lateral %.3f m, heading change %.3f rad; "
             "cmd vx=-0.30 m/s for 3 s: %.2f m; max tilt %.3f rad",
             fwd, fwd / 4.0, lat, dyaw, back, rec.maxTilt()));
  }

  // ---------------- 3. turning -----------------------------------------------
  {
    rec.setTest("3_turn");
    const Truth a = rec.truth();
    walkFor(robot, {0, 0, 0.8}, 4.0);
    const double left = rec.yawTravelled();
    rec.setTest("3_turn");
    walkFor(robot, {0, 0, -0.8}, 4.0);
    const double right = rec.yawTravelled();
    const Truth b = rec.truth();
    const double drift = std::hypot(b.x - a.x, b.y - a.y);
    const bool pass = left > 0.6 * 0.8 * 4.0 && right < -0.6 * 0.8 * 4.0 && drift < 0.35;
    report(3, "Robot turns correctly", pass,
           f("cmd wz=+0.8 rad/s 4 s -> yaw %+.2f rad (left); cmd wz=-0.8 rad/s 4 s -> yaw %+.2f rad (right); "
             "position drift during turns %.3f m",
             left, right, drift));
  }

  // ---------------- 4. unobstructed goal -------------------------------------
  {
    rec.setTest("4_goal_free");
    const double gx = 0.0, gy = 2.5;
    const NavResult r = nav.goTo(gx, gy);
    const Truth t = rec.truth();
    const double err = std::hypot(t.x - gx, t.y - gy);
    const bool pass = r.success && err < 0.2 && rec.minClearance() > 0.0;
    report(4, "Robot reaches a destination in free space", pass,
           f("goTo(%.1f, %.1f): nav=%s (%s) in %.1f s; MEASURED final error %.3f m; min footprint clearance %.2f m",
             gx, gy, r.success ? "SUCCESS" : "FAILURE", r.reason.c_str(), r.elapsed, err, rec.minClearance()));
    // return to A, facing +x (towards the obstacle course)
    rec.setTest("4b_return_to_A");
    nav.goTo(0.0, 0.0, 0.0);
  }

  // ---------------- 5. obstacle detection --------------------------------------
  const double Ax = 0.0, Ay = 0.0, Bx = 5.0, By = 0.0;
  {
    rec.setTest("5_detect");
    sleepS(1.5);  // a few scans from A
    const auto occ = nav.grid().occupancySnapshot();
    const auto& gc = nav.grid().config();
    int wall_cells = 0, false_cells = 0;
    for (int y = 0; y < gc.height; ++y)
      for (int x = 0; x < gc.width; ++x) {
        if (occ[y * gc.width + x] != 2) continue;
        double wx, wy;
        nav.grid().cellToWorld(x, y, wx, wy);
        const double dob = pointObstacleDistance(wx, wy);
        if (std::fabs(wx - 2.5) < 0.3 && std::fabs(wy) < 0.7) ++wall_cells;
        if (dob > 0.15) ++false_cells;
      }
    const auto cost = nav.grid().costSnapshot();
    const bool straight_blocked = !PathPlanner::segmentFree(nav.grid(), cost, Ax, Ay, Bx, By, 254);
    const Truth t = rec.truth();
    const bool pass = straight_blocked && wall_cells >= 10;
    report(5, "Robot detects an obstacle blocking its path", pass,
           f("robot at (%.2f, %.2f); scan-built map has %d occupied cells on the wall (x=2.3..2.7); straight line "
             "A->B is %s in the cost map; %d occupied cells > 0.15 m from any real obstacle",
             t.x, t.y, wall_cells, straight_blocked ? "BLOCKED" : "free", false_cells));
    dumpMap(nav, L("map_at_A.csv"));
  }

  // ---------------- 6 + 7. navigate around the obstacle to B ---------------------
  {
    rec.setTest("6_avoid");
    rec.clearTraj();
    const Truth a = rec.truth();
    const NavResult r = nav.goTo(Bx, By);
    const Truth t = rec.truth();
    const double err = std::hypot(t.x - Bx, t.y - By);
    const double lat = rec.lateralMax(Ax, Ay, 0.0);
    const double clear = rec.minClearance();
    dumpPath(nav.telemetry().path, L("last_path.csv"));
    dumpMap(nav, L("map_final.csv"));
    report(6, "Robot navigates around the obstacle", clear > 0.0 && lat > 0.6 && t.x > 3.0,
           f("start (%.2f, %.2f); max lateral excursion from line A-B %.2f m; minimum measured footprint clearance "
             "to any obstacle %.3f m (%s); replans %d, safety stops %d; closest scan return %.2f m",
             a.x, a.y, lat, clear, clear > 0 ? "no collision" : "COLLISION", r.replans, r.safety_stops,
             r.min_obstacle_range));
    report(7, "Robot successfully reaches the destination", r.success && err < 0.2 && clear > 0.0,
           f("goTo(%.1f, %.1f): nav=%s (%s) in %.1f s, travelled %.2f m; MEASURED final position (%.3f, %.3f), "
             "error %.3f m",
             Bx, By, r.success ? "SUCCESS" : "FAILURE", r.reason.c_str(), r.elapsed, r.distance_travelled, t.x, t.y,
             err));
  }

  // ---------------- 8. success / failure reporting from feedback ------------------
  {
    rec.setTest("8_unreachable");
    // (a) the success reported in test 7 must agree with ground truth (checked above);
    // (b) an impossible goal must be reported as FAILURE, without collision.
    const NavResult r = nav.goTo(4.0, 1.1);  // centre of the pillar
    const double clear = rec.minClearance();
    const bool agree7 = g_results.size() >= 7 && g_results[6].pass;
    const bool pass = !r.success && clear > 0.0 && agree7;
    report(8, "Application reports success / failure from feedback", pass,
           f("unreachable goal (pillar centre): nav=%s, reason \"%s\" after %.1f s, min clearance %.3f m; "
             "test-7 nav SUCCESS confirmed by measured pose: %s",
             r.success ? "SUCCESS (wrong!)" : "FAILURE (correct)", r.reason.c_str(), r.elapsed, clear,
             agree7 ? "yes" : "no"));
  }

  robot.setWalking(false);
  waitMode(robot, LocoMode::STAND, 5.0);
  nav.shutdown();

  int passed = 0;
  std::printf("\n================ ACCEPTANCE SUMMARY ================\n");
  for (auto& r : g_results) {
    std::printf("  [%s] %d. %s\n", r.pass ? "PASS" : "FAIL", r.id, r.name.c_str());
    passed += r.pass;
  }
  const double wall = std::chrono::duration<double>(clk::now() - t_start).count();
  const double sim = (robot.lastState().tick - tick_start) * 1e-3;
  std::printf("  %d / %zu passed\n  control loop: %lu cycles, %lu late (>4 ms, %.2f %%), max jitter %.1f ms | lowcmd sent %lu"
              " | scans %lu | sim/wall time %.1f/%.1f s (RTF %.2f)\n",
              passed, g_results.size(), robot.controlCycles(), robot.lateCycles(),
              100.0 * robot.lateCycles() / std::max(1ul, robot.controlCycles()), robot.maxLoopJitterMs(),
              robot.io().lowcmdCount(), robot.io().scanCount(), sim, wall, sim / wall);
  if (!log_dir.empty()) {
    std::ofstream o(L("summary.txt"));
    for (auto& r : g_results) o << (r.pass ? "PASS" : "FAIL") << " | " << r.id << ". " << r.name << " | " << r.detail << "\n";
  }
  robot.stop();
  return passed == (int)g_results.size() ? 0 : 4;
}
