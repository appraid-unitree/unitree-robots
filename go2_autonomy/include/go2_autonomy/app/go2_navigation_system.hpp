// Single entry point for business applications:
//
//   go2::Go2NavigationSystem robot;            // sim defaults (domain 1, "lo")
//   robot.start();                             // DDS up, control loop running
//   robot.standUp();
//   go2::NavResult r = robot.goTo(5.0, 0.0);   // blocking, obstacle-aware
//   if (r.success) ...                         // judged on measured pose
//   robot.sitDown();
#pragma once

#include <memory>
#include <optional>

#include "go2_autonomy/app/go2_robot.hpp"
#include "go2_autonomy/navigation/navigator.hpp"

namespace go2 {

struct SystemConfig {
  RobotConfig robot;
  NavConfig nav;
  GridConfig grid;
  PlannerConfig planner;
};

class Go2NavigationSystem {
 public:
  explicit Go2NavigationSystem(const SystemConfig& cfg = SystemConfig());
  ~Go2NavigationSystem();

  bool start(double wait_state_timeout_s = 10.0);  // DDS + 500 Hz control + 20 Hz navigation
  bool standUp(double timeout_s = 20.0);
  bool sitDown(double timeout_s = 8.0);
  void shutdown();

  NavResult goTo(double x, double y, std::optional<double> yaw = std::nullopt);
  void cancel() { nav_->cancel(); }

  EstimatedState pose() const { return robot_->estimate(); }
  Go2Robot& robot() { return *robot_; }
  Navigator& navigator() { return *nav_; }

 private:
  SystemConfig cfg_;
  std::unique_ptr<Go2Robot> robot_;
  std::unique_ptr<Navigator> nav_;
};

}  // namespace go2
