// Autonomous point-to-point navigation:  goTo(x, y [, yaw]).
//
// 20 Hz loop:  scan -> occupancy grid -> A* (re)plan -> pure-pursuit path
// following -> body twist command -> locomotion. A reactive safety layer
// checks the raw scan in the body frame and vetoes forward motion when an
// obstacle is inside the stopping zone (triggering an immediate replan).
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "go2_autonomy/navigation/robot_interface.hpp"
#include "go2_autonomy/perception/occupancy_grid.hpp"
#include "go2_autonomy/planning/path_planner.hpp"

namespace go2 {

struct NavConfig {
  double rate_hz = 20.0;
  double goal_tolerance = 0.15;      // [m]
  double yaw_tolerance = 0.12;       // [rad] (only if a goal yaw is given)
  double v_max = 0.5;                // [m/s]
  double v_min = 0.10;
  double w_max = 1.0;                // [rad/s]
  double lookahead = 0.55;           // pure pursuit [m]
  double heading_gain = 1.8;
  double rotate_in_place_angle = 0.75;  // [rad] larger heading errors: turn on the spot
  double replan_period = 1.0;        // [s]
  double stop_zone = 0.30;           // reactive stop distance in front of the body [m]
  double body_half_length = 0.36, body_half_width = 0.20;
  double timeout = 180.0;            // [s]
  double no_progress_timeout = 25.0; // [s] without getting 0.25 m closer -> fail
  double plan_fail_timeout = 10.0;   // [s] without any valid plan -> fail
  bool require_scan = true;          // refuse to move without fresh range data
  double scan_timeout = 0.5;         // [s] scan older than this -> stop
  double sensor_fail_timeout = 2.0;  // [s] stopped for missing scan / pose -> fail
};

enum class NavStatus { IDLE, PLANNING, FOLLOWING, ROTATING, BLOCKED, ALIGNING, SUCCEEDED, FAILED };
const char* toString(NavStatus s);

struct NavResult {
  bool success = false;
  std::string reason;
  Pose2D final_pose;
  double position_error = 0.0;  // measured distance to goal [m]
  double yaw_error = 0.0;
  double elapsed = 0.0;         // [s]
  double distance_travelled = 0.0;
  int replans = 0;
  int safety_stops = 0;
  double min_obstacle_range = 1e9;  // closest scan obstacle point seen (body frame, planar)
};

struct NavTelemetry {
  NavStatus status = NavStatus::IDLE;
  Pose2D pose;
  Twist2D cmd;
  Path path;
  double dist_to_goal = 0;
  double min_front_range = 1e9;
  int occupied_cells = 0;
  ScanStats last_scan;
};

class Navigator {
 public:
  Navigator(NavRobotInterface& robot, const NavConfig& cfg = NavConfig(), const GridConfig& grid = GridConfig(),
            const PlannerConfig& planner = PlannerConfig());
  ~Navigator();

  void start();  // launches the navigation thread (mapping runs continuously)
  void shutdown();

  // Blocking. Returns when the goal is reached, navigation fails, or timeout.
  NavResult goTo(double x, double y, std::optional<double> yaw = std::nullopt);
  // Non-blocking variants.
  void goToAsync(double x, double y, std::optional<double> yaw = std::nullopt);
  void cancel();
  bool waitForResult(NavResult& out, double timeout_s);

  NavTelemetry telemetry() const;
  const OccupancyGrid& grid() const { return grid_; }
  void setLogger(std::function<void(const std::string&)> f) { log_ = std::move(f); }

 private:
  void loop();
  void step(double dt);
  void finish(bool ok, const std::string& reason, const EstimatedState& e);
  // Queue a log line (caller holds m_); flushed by step() after unlocking, so a
  // user logger may safely call telemetry() / cancel().
  void log(const std::string& s) { pending_logs_.push_back(s); }
  void flushLogs();
  void stopRobot() { robot_.setVelocity({}); robot_.setWalking(false); }

  NavRobotInterface& robot_;
  NavConfig cfg_;
  OccupancyGrid grid_;
  PathPlanner planner_;

  std::thread th_;
  std::atomic<bool> run_{false};

  mutable std::mutex m_;
  std::condition_variable cv_;
  bool active_ = false, done_ = false;
  double gx_ = 0, gy_ = 0;
  std::optional<double> gyaw_;
  NavResult result_;
  NavTelemetry tel_;

  // loop state
  unsigned long scan_seq_ = 0;
  Path path_;
  double t_goal_start_ = 0, t_last_plan_ = -1e9, t_last_valid_plan_ = 0, t_progress_ = 0;
  double best_dist_ = 1e9;
  Pose2D last_pose_;
  bool have_last_pose_ = false;
  bool force_replan_ = false;
  bool blocked_latched_ = false;
  double last_scan_time_ = -1e9, t_pose_ok_ = 0, t_sensor_ok_ = 0;
  unsigned long goal_id_ = 0;
  std::vector<std::string> pending_logs_;
  NavResult acc_;
  std::function<void(const std::string&)> log_;
};

}  // namespace go2
