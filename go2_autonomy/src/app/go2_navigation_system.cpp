#include "go2_autonomy/app/go2_navigation_system.hpp"

#include <chrono>
#include <thread>

namespace go2 {

Go2NavigationSystem::Go2NavigationSystem(const SystemConfig& cfg)
    : cfg_(cfg), robot_(new Go2Robot(cfg.robot)), nav_(new Navigator(*robot_, cfg.nav, cfg.grid, cfg.planner)) {}

Go2NavigationSystem::~Go2NavigationSystem() { shutdown(); }

bool Go2NavigationSystem::start(double wait_state_timeout_s) { return robot_->start(wait_state_timeout_s); }

bool Go2NavigationSystem::standUp(double timeout_s) {
  if (!robot_->standUp(timeout_s)) return false;
  nav_->start();  // begin mapping once the robot stands still
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // a few scans into the map
  return true;
}

bool Go2NavigationSystem::sitDown(double timeout_s) { return robot_->sitDown(timeout_s); }

void Go2NavigationSystem::shutdown() {
  if (nav_) nav_->shutdown();
  if (robot_) robot_->stop();
}

NavResult Go2NavigationSystem::goTo(double x, double y, std::optional<double> yaw) { return nav_->goTo(x, y, yaw); }

}  // namespace go2
