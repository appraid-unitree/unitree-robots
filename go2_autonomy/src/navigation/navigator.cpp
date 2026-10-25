#include "go2_autonomy/navigation/navigator.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

#include "go2_autonomy/comm/go2_dds_interface.hpp"  // monotonicSeconds()

namespace go2 {

const char* toString(NavStatus s) {
  switch (s) {
    case NavStatus::IDLE: return "IDLE";
    case NavStatus::PLANNING: return "PLANNING";
    case NavStatus::FOLLOWING: return "FOLLOWING";
    case NavStatus::ROTATING: return "ROTATING";
    case NavStatus::BLOCKED: return "BLOCKED";
    case NavStatus::ALIGNING: return "ALIGNING";
    case NavStatus::SUCCEEDED: return "SUCCEEDED";
    case NavStatus::FAILED: return "FAILED";
  }
  return "?";
}

namespace {
std::string fmt(const char* f, double a = 0, double b = 0, double c = 0, double d = 0) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), f, a, b, c, d);
  return buf;
}
}  // namespace

Navigator::Navigator(NavRobotInterface& robot, const NavConfig& cfg, const GridConfig& grid,
                     const PlannerConfig& planner)
    : robot_(robot), cfg_(cfg), grid_(grid), planner_(planner) {}

Navigator::~Navigator() { shutdown(); }

void Navigator::start() {
  if (run_.exchange(true)) return;
  th_ = std::thread(&Navigator::loop, this);
}

void Navigator::shutdown() {
  cancel();  // an active goal must never be left with the robot walking
  if (!run_.exchange(false)) return;
  if (th_.joinable()) th_.join();
  flushLogs();
}

void Navigator::flushLogs() {
  std::vector<std::string> lines;
  {
    std::lock_guard<std::mutex> l(m_);
    lines.swap(pending_logs_);
  }
  if (log_)
    for (const auto& s : lines) log_(s);
}

void Navigator::goToAsync(double x, double y, std::optional<double> yaw) {
  std::lock_guard<std::mutex> l(m_);
  gx_ = x;
  gy_ = y;
  gyaw_ = yaw;
  active_ = true;
  done_ = false;
  result_ = NavResult{};
  acc_ = NavResult{};
  path_ = Path{};
  t_goal_start_ = monotonicSeconds();
  t_last_plan_ = -1e9;
  t_last_valid_plan_ = t_goal_start_;
  t_progress_ = t_goal_start_;
  best_dist_ = 1e9;
  have_last_pose_ = false;
  force_replan_ = true;
  t_pose_ok_ = t_sensor_ok_ = t_goal_start_;
  ++goal_id_;
  tel_.status = NavStatus::PLANNING;
}

void Navigator::cancel() {
  std::lock_guard<std::mutex> l(m_);
  if (!active_) return;
  active_ = false;
  done_ = true;
  result_ = acc_;
  result_.success = false;
  result_.reason = "cancelled";
  result_.final_pose = tel_.pose;
  result_.position_error = std::hypot(gx_ - tel_.pose.x, gy_ - tel_.pose.y);
  result_.elapsed = monotonicSeconds() - t_goal_start_;
  tel_.status = NavStatus::IDLE;
  stopRobot();
  cv_.notify_all();
}

bool Navigator::waitForResult(NavResult& out, double timeout_s) {
  std::unique_lock<std::mutex> l(m_);
  const bool ok = cv_.wait_for(l, std::chrono::duration<double>(timeout_s), [&] { return done_; });
  if (ok) out = result_;
  return ok;
}

NavResult Navigator::goTo(double x, double y, std::optional<double> yaw) {
  goToAsync(x, y, yaw);
  NavResult r;
  if (!waitForResult(r, cfg_.timeout + 30.0)) {
    cancel();
    r.success = false;
    r.reason = "waitForResult timeout";
  }
  return r;
}

NavTelemetry Navigator::telemetry() const {
  std::lock_guard<std::mutex> l(m_);
  return tel_;
}

void Navigator::loop() {
  const auto period = std::chrono::duration<double>(1.0 / cfg_.rate_hz);
  auto next = std::chrono::steady_clock::now();
  while (run_.load()) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    step(1.0 / cfg_.rate_hz);
    flushLogs();
    std::this_thread::sleep_until(next);
  }
}

void Navigator::finish(bool ok, const std::string& reason, const EstimatedState& e) {
  // caller holds m_
  active_ = false;
  done_ = true;
  result_ = acc_;
  result_.success = ok;
  result_.reason = reason;
  result_.final_pose = e.pose;
  result_.position_error = std::hypot(gx_ - e.pose.x, gy_ - e.pose.y);
  result_.yaw_error = gyaw_ ? wrapAngle(*gyaw_ - e.pose.yaw) : 0.0;
  result_.elapsed = monotonicSeconds() - t_goal_start_;
  tel_.status = ok ? NavStatus::SUCCEEDED : NavStatus::FAILED;
  log(std::string(ok ? "[nav] SUCCEEDED: " : "[nav] FAILED: ") + reason +
      fmt(" | final=(%.3f, %.3f) err=%.3f m, t=%.1f s", e.pose.x, e.pose.y, result_.position_error, result_.elapsed));
  cv_.notify_all();
}

void Navigator::step(double /*dt*/) {
  const double now = monotonicSeconds();
  const EstimatedState e = robot_.estimate();
  if (!e.valid) {
    // Pose missing or stale: never drive blind.
    std::lock_guard<std::mutex> l(m_);
    if (!active_) return;
    stopRobot();
    tel_.status = NavStatus::BLOCKED;
    tel_.cmd = {};
    if (now - t_pose_ok_ > cfg_.sensor_fail_timeout) finish(false, "pose estimate unavailable or stale", e);
    return;
  }

  // ---------------- perception -----------------------------------------
  PointCloud pc;
  double min_front = 1e9, min_any = 1e9;
  bool new_scan = false;
  if (robot_.latestScan(pc, scan_seq_)) {
    new_scan = true;
    scan_seq_ = pc.seq;
    last_scan_time_ = pc.receive_time > 0 ? pc.receive_time : now;
    const double base_h = e.height + robot_.footRadius();
    const ScanStats st = grid_.integrate(pc, e.pose, e.roll, e.pitch, base_h);
    grid_.updateCostmap();
    const auto& gc = grid_.config();
    const double cr = std::cos(e.roll), sr = std::sin(e.roll), cp = std::cos(e.pitch), sp = std::sin(e.pitch);
    for (const Vec3& p : pc.points) {
      if (std::fabs(p.x) < gc.self_filter_x && std::fabs(p.y) < gc.self_filter_y) continue;
      const double y1 = cr * p.y - sr * p.z, z1 = sr * p.y + cr * p.z;
      const double xl = cp * p.x + sp * z1, zl = -sp * p.x + cp * z1;
      const double z = base_h + zl;
      if (z < gc.obstacle_min_z || z > gc.obstacle_max_z) continue;
      min_any = std::min(min_any, std::hypot(xl, y1));
      if (xl > 0 && std::fabs(y1) < cfg_.body_half_width + 0.05) min_front = std::min(min_front, xl - cfg_.body_half_length);
    }
    std::lock_guard<std::mutex> l(m_);
    tel_.last_scan = st;
    tel_.min_front_range = min_front;
  }

  std::unique_lock<std::mutex> l(m_);
  tel_.pose = e.pose;
  if (!active_) return;
  t_pose_ok_ = now;

  // Range data must be fresh, otherwise neither the map nor the stop zone can be trusted.
  if (cfg_.require_scan && now - last_scan_time_ > cfg_.scan_timeout) {
    stopRobot();
    tel_.status = NavStatus::BLOCKED;
    tel_.cmd = {};
    if (now - t_sensor_ok_ > cfg_.sensor_fail_timeout)
      finish(false, last_scan_time_ < 0 ? "no range-sensor data received" : "range-sensor data stale", e);
    return;
  }
  t_sensor_ok_ = now;

  if (new_scan) acc_.min_obstacle_range = std::min(acc_.min_obstacle_range, min_any);
  if (have_last_pose_) acc_.distance_travelled += std::hypot(e.pose.x - last_pose_.x, e.pose.y - last_pose_.y);
  last_pose_ = e.pose;
  have_last_pose_ = true;

  const double d = std::hypot(gx_ - e.pose.x, gy_ - e.pose.y);
  tel_.dist_to_goal = d;
  if (d < best_dist_ - 0.25) { best_dist_ = d; t_progress_ = now; }

  if (now - t_goal_start_ > cfg_.timeout) {
    robot_.setVelocity({});
    robot_.setWalking(false);
    finish(false, "timeout", e);
    return;
  }
  if (!robot_.isReady()) {
    stopRobot();
    finish(false, "locomotion not ready (fault or not standing)", e);
    return;
  }

  Twist2D cmd;
  const double arrive_radius = 0.6 * cfg_.goal_tolerance;
  // ---------------- arrival / alignment / stopping ------------------------
  if (d < arrive_radius || tel_.status == NavStatus::ALIGNING) {
    double yaw_err = gyaw_ ? wrapAngle(*gyaw_ - e.pose.yaw) : 0.0;
    if (gyaw_ && std::fabs(yaw_err) > cfg_.yaw_tolerance && d < cfg_.goal_tolerance) {
      tel_.status = NavStatus::ALIGNING;
      cmd.wz = clamp(1.5 * yaw_err, -cfg_.w_max, cfg_.w_max);
      if (std::fabs(cmd.wz) < 0.25) cmd.wz = std::copysign(0.25, yaw_err);
      robot_.setWalking(true);
      robot_.setVelocity(cmd);
      tel_.cmd = cmd;
      return;
    }
    if (d < cfg_.goal_tolerance) {
      // Stop and wait for the gait to settle into STAND, then judge the
      // result on the MEASURED pose.
      robot_.setVelocity({});
      robot_.setWalking(false);
      tel_.cmd = {};
      if (!robot_.isWalking()) {
        const double err = std::hypot(gx_ - e.pose.x, gy_ - e.pose.y);
        const double yerr = gyaw_ ? std::fabs(wrapAngle(*gyaw_ - e.pose.yaw)) : 0.0;
        const bool ok = err <= cfg_.goal_tolerance && (!gyaw_ || yerr <= 1.5 * cfg_.yaw_tolerance);
        finish(ok, ok ? "goal reached (measured)" : "stopped outside tolerance", e);
      }
      return;
    }
    tel_.status = NavStatus::FOLLOWING;  // drifted out while aligning
  }

  // ---------------- planning ----------------------------------------------
  const std::vector<uint8_t> cost = grid_.costSnapshot();
  bool blocked = false;
  if (path_.valid) {
    // Sample the remaining path; cells within 0.4 m of the robot are ignored
    // (it is already there, e.g. while escaping an inflation halo).
    const double res = grid_.config().resolution;
    const int W = grid_.config().width;
    for (size_t i = 1; i < path_.waypoints.size() && !blocked; ++i) {
      const Vec3& a = path_.waypoints[i - 1];
      const Vec3& b = path_.waypoints[i];
      const int n = std::max(1, (int)std::ceil(std::hypot(b.x - a.x, b.y - a.y) / (0.5 * res)));
      for (int k = 0; k <= n; ++k) {
        const double x = a.x + (b.x - a.x) * k / n, y = a.y + (b.y - a.y) * k / n;
        if (std::hypot(x - e.pose.x, y - e.pose.y) < 0.4) continue;
        int cx, cy;
        if (!grid_.worldToCell(x, y, cx, cy) || cost[cy * W + cx] >= 254) { blocked = true; break; }
      }
    }
  }
  if (!path_.valid || force_replan_ || blocked || now - t_last_plan_ > cfg_.replan_period) {
    // Plan WITHOUT holding m_ so telemetry() / cancel() stay responsive.
    const unsigned long gid = goal_id_;
    const double gx = gx_, gy = gy_;
    l.unlock();
    Path p = planner_.plan(grid_, cost, e.pose.x, e.pose.y, gx, gy);
    l.lock();
    if (!active_ || gid != goal_id_) return;  // cancelled or replaced meanwhile
    t_last_plan_ = now;
    const bool was_valid = path_.valid;
    if (p.valid) {
      if (blocked || force_replan_ || !was_valid)
        log(fmt("[nav] plan: %.0f waypoints, %.2f m", (double)p.waypoints.size(), p.length) +
            (blocked ? " (previous path blocked by new obstacle)" : ""));
      path_ = p;
      t_last_valid_plan_ = now;
      acc_.replans++;
    } else {
      path_ = Path{};
      if (now - t_last_valid_plan_ > cfg_.plan_fail_timeout) {
        robot_.setVelocity({});
        robot_.setWalking(false);
        finish(false, "no collision-free path to goal", e);
        return;
      }
    }
    force_replan_ = false;
  }
  tel_.path = path_;

  if (now - t_progress_ > cfg_.no_progress_timeout) {
    robot_.setVelocity({});
    robot_.setWalking(false);
    finish(false, "no progress towards goal", e);
    return;
  }

  if (!path_.valid) {
    tel_.status = NavStatus::BLOCKED;
    robot_.setVelocity({});
    robot_.setWalking(true);  // keep stepping in place while waiting for a plan
    tel_.cmd = {};
    return;
  }

  // ---------------- pure pursuit --------------------------------------------
  const auto& wp = path_.waypoints;
  // closest point on the polyline
  size_t seg = 0;
  double best = 1e18, best_t = 0;
  for (size_t i = 1; i < wp.size(); ++i) {
    const double ax = wp[i - 1].x, ay = wp[i - 1].y, bx = wp[i].x, by = wp[i].y;
    const double vx = bx - ax, vy = by - ay, L2 = vx * vx + vy * vy;
    double t = L2 > 1e-9 ? ((e.pose.x - ax) * vx + (e.pose.y - ay) * vy) / L2 : 0.0;
    t = clamp(t, 0.0, 1.0);
    const double px = ax + t * vx, py = ay + t * vy;
    const double dd = std::hypot(e.pose.x - px, e.pose.y - py);
    if (dd < best) { best = dd; seg = i; best_t = t; }
  }
  // walk forward along the path by `lookahead`
  double rem = cfg_.lookahead;
  double lx = wp.back().x, ly = wp.back().y;
  {
    double ax = wp[seg - 1].x + best_t * (wp[seg].x - wp[seg - 1].x);
    double ay = wp[seg - 1].y + best_t * (wp[seg].y - wp[seg - 1].y);
    for (size_t i = seg; i < wp.size(); ++i) {
      const double sl = std::hypot(wp[i].x - ax, wp[i].y - ay);
      if (sl >= rem) {
        lx = ax + (wp[i].x - ax) * rem / sl;
        ly = ay + (wp[i].y - ay) * rem / sl;
        break;
      }
      rem -= sl;
      ax = wp[i].x;
      ay = wp[i].y;
    }
  }
  const double target_heading = std::atan2(ly - e.pose.y, lx - e.pose.x);
  const double err = wrapAngle(target_heading - e.pose.yaw);

  if (std::fabs(err) > cfg_.rotate_in_place_angle) {
    tel_.status = NavStatus::ROTATING;
    cmd.vx = 0.0;
    cmd.wz = clamp(cfg_.heading_gain * err, -cfg_.w_max, cfg_.w_max);
  } else {
    tel_.status = NavStatus::FOLLOWING;
    double v = cfg_.v_max * (1.0 - 0.7 * std::fabs(err) / cfg_.rotate_in_place_angle);
    const auto& gc = grid_.config();
    const double c = grid_.clearance(e.pose.x, e.pose.y);
    v *= clamp((c - gc.lethal_radius) / (gc.soft_radius - gc.lethal_radius), 0.45, 1.0);
    v = std::min(v, std::max(cfg_.v_min, 0.8 * d));
    cmd.vx = clamp(v, 0.0, cfg_.v_max);
    cmd.wz = clamp(cfg_.heading_gain * err, -cfg_.w_max, cfg_.w_max);
  }

  // ---------------- reactive safety layer -----------------------------------
  const double front = tel_.min_front_range;
  if (front < cfg_.stop_zone && cmd.vx > 0.0) {
    cmd.vx = 0.0;
    if (!blocked_latched_) {
      blocked_latched_ = true;
      acc_.safety_stops++;
      force_replan_ = true;
      log(fmt("[nav] safety stop: obstacle %.2f m ahead of the body, replanning", front));
    }
    if (std::fabs(cmd.wz) < 0.3) cmd.wz = std::copysign(0.3, err == 0 ? 1.0 : err);
  } else if (front > cfg_.stop_zone + 0.1) {
    blocked_latched_ = false;
  }

  robot_.setWalking(true);
  robot_.setVelocity(cmd);
  tel_.cmd = cmd;
}

}  // namespace go2
