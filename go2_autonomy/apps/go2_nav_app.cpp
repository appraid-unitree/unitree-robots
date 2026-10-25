// go2_nav_app -- business-level example application ("robot.goTo(x, y)").
//
//   ./go2_nav_app --goal 5 0                 # A (current pose) -> B (5, 0)
//   ./go2_nav_app --goal 2 1 --goal 0 0 1.57 # multi-goal mission, final yaw
//
// Options:
//   --iface <name>      DDS network interface (default lo)
//   --domain <id>       DDS domain (default 1 = unitree_mujoco)
//   --pose sport|odom   navigation pose source (default sport = rt/sportmodestate)
//   --vmax <m/s>        max forward speed (default 0.5)
//   --goal x y [yaw]    add a goal (world/odometry frame), repeatable
//   --sit               sit down after the mission
//
// REAL ROBOT: do not point this at a physical Go2 before reading
// docs/HARDWARE_NOTES.md (sport service must be released, the low-level gait
// is unvalidated on hardware, and rt/sportmodestate is not available).
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "go2_autonomy/app/go2_navigation_system.hpp"

using namespace go2;

struct Goal {
  double x, y;
  std::optional<double> yaw;
};

static std::atomic<bool> g_interrupted{false};
static void onSigint(int) { g_interrupted.store(true); }

static bool isNumber(const char* s) {
  char* end = nullptr;
  std::strtod(s, &end);
  return end != s && end && *end == '\0';
}

int main(int argc, char** argv) {
  SystemConfig cfg;
  std::vector<Goal> goals;
  bool sit = false;
  bool real_robot_ack = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--iface" && i + 1 < argc) cfg.robot.dds.network_interface = argv[++i];
    else if (a == "--domain" && i + 1 < argc) cfg.robot.dds.domain_id = std::atoi(argv[++i]);
    else if (a == "--pose" && i + 1 < argc) {
      std::string p = argv[++i];
      cfg.robot.pose_source = (p == "odom") ? PoseSource::LEG_ODOMETRY : PoseSource::SPORT_STATE;
    } else if (a == "--vmax" && i + 1 < argc) cfg.nav.v_max = std::atof(argv[++i]);
    else if (a == "--goal" && i + 2 < argc) {
      Goal g{std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::nullopt};
      i += 2;
      if (i + 1 < argc && isNumber(argv[i + 1])) g.yaw = std::atof(argv[++i]);
      goals.push_back(g);
    } else if (a == "--sit") sit = true;
    else if (a == "--real-robot-i-have-read-hardware-notes") real_robot_ack = true;
    else { std::cerr << "unknown option " << a << std::endl; return 2; }
  }
  if (goals.empty()) goals.push_back({5.0, 0.0, std::nullopt});
  if (cfg.robot.dds.domain_id == 0 && !real_robot_ack) {
    // DDS domain 0 is the real Go2. Our low-level gait is NOT validated on hardware.
    std::cerr << "[app] refusing to run on DDS domain 0 (real robot). Read docs/HARDWARE_NOTES.md first; "
                 "override with --real-robot-i-have-read-hardware-notes" << std::endl;
    return 2;
  }

  std::signal(SIGINT, onSigint);  // Ctrl+C: cancel the goal, stop walking, damp, exit
  Go2NavigationSystem robot(cfg);
  robot.navigator().setLogger([](const std::string& s) { std::cout << s << std::endl; });
  std::cout << "[app] connecting (domain " << cfg.robot.dds.domain_id << ", iface " << cfg.robot.dds.network_interface
            << ")" << std::endl;
  if (!robot.start()) return 1;
  std::cout << "[app] rt/lowstate OK, standing up..." << std::endl;
  if (!robot.standUp()) { std::cerr << "[app] stand-up failed" << std::endl; return 1; }
  std::cout << "[app] standing. pose source: "
            << (cfg.robot.pose_source == PoseSource::SPORT_STATE ? "rt/sportmodestate" : "leg odometry") << std::endl;

  std::atomic<bool> monitor{true};
  std::thread mon([&] {
    while (monitor.load()) {
      if (g_interrupted.load()) { robot.cancel(); }
      const NavTelemetry t = robot.navigator().telemetry();
      std::printf("[status] %-9s pose=(%6.2f,%6.2f,%5.2f) cmd=(%.2f,%.2f) goal_dist=%.2f front=%.2f loco=%s\n",
                  toString(t.status), t.pose.x, t.pose.y, t.pose.yaw, t.cmd.vx, t.cmd.wz, t.dist_to_goal,
                  t.min_front_range > 1e8 ? -1.0 : t.min_front_range, toString(robot.robot().locomotion().mode()));
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  int failures = 0;
  for (const Goal& g : goals) {
    std::printf("[app] goTo(%.2f, %.2f%s)\n", g.x, g.y, g.yaw ? ", yaw" : "");
    const NavResult r = robot.goTo(g.x, g.y, g.yaw);
    std::printf("[app] RESULT %s: %s | final=(%.3f, %.3f, %.3f) pos_err=%.3f m yaw_err=%.3f rad time=%.1f s "
                "travelled=%.2f m replans=%d safety_stops=%d closest_obstacle=%.2f m\n",
                r.success ? "SUCCESS" : "FAILURE", r.reason.c_str(), r.final_pose.x, r.final_pose.y, r.final_pose.yaw,
                r.position_error, r.yaw_error, r.elapsed, r.distance_travelled, r.replans, r.safety_stops,
                r.min_obstacle_range);
    if (!r.success || g_interrupted.load()) { ++failures; break; }
  }
  monitor.store(false);
  mon.join();
  if (sit) robot.sitDown();
  robot.shutdown();
  std::fflush(stdout);
  return failures == 0 ? 0 : 3;
}
