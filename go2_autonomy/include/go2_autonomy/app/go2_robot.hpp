// Go2Robot: wires DDS I/O, state estimation and the locomotion controller
// into a 500 Hz real-time loop, and exposes a small high-level API.
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <thread>

#include "go2_autonomy/comm/go2_dds_interface.hpp"
#include "go2_autonomy/estimation/state_estimator.hpp"
#include "go2_autonomy/locomotion/locomotion_controller.hpp"
#include "go2_autonomy/navigation/robot_interface.hpp"

namespace go2 {

struct RobotConfig {
  DdsConfig dds;
  GaitParams gait;
  PoseSource pose_source = PoseSource::SPORT_STATE;  // pose used by navigation
  bool gait_velocity_feedback = true;  // feed leg-odometry body velocity into the gait (proprioceptive)
  double control_dt = 0.002;           // 500 Hz
  double lowstate_timeout = 0.25;      // [s] -> damping if exceeded while active
};

class Go2Robot : public NavRobotInterface {
 public:
  explicit Go2Robot(const RobotConfig& cfg = RobotConfig());
  ~Go2Robot() override;

  // Initialise DDS, wait for rt/lowstate, start the control thread.
  bool start(double wait_state_timeout_s = 10.0);
  // Safe stop: finish walking (-> STAND), command damping for ~0.3 s, then
  // terminate and JOIN the control thread. The last command left in the
  // motor drivers / simulator bridge is therefore always kp = 0 damping.
  void stop();

  // Blocking helpers (return false on timeout / fault).
  bool standUp(double timeout_s = 15.0);
  bool sitDown(double timeout_s = 5.0);
  void damping() { loco_.requestDamping(); }

  // NavRobotInterface
  EstimatedState estimate() const override { return est_.estimate(); }
  bool latestScan(PointCloud& out, unsigned long newer_than_seq) const override {
    return io_.latestScan(out, newer_than_seq);
  }
  void setVelocity(const Twist2D& v) override { loco_.setVelocityCommand(v); }
  void setWalking(bool walk) override { loco_.requestWalk(walk); }
  bool isReady() const override { return loco_.isReady() && !fault_.load(); }
  bool isWalking() const override { return loco_.mode() == LocoMode::WALK; }

  LocomotionController& locomotion() { return loco_; }
  StateEstimator& estimator() { return est_; }
  Go2DdsInterface& io() { return io_; }
  const RobotConfig& config() const { return cfg_; }
  bool fault() const { return fault_.load(); }
  LowLevelState lastState() const;
  unsigned long controlCycles() const { return cycles_.load(); }
  double maxLoopJitterMs() const { return max_jitter_ms_.load(); }
  unsigned long lateCycles() const { return late_cycles_.load(); }  // period > 2x nominal

 private:
  void controlStep();
  void controlLoop();

  RobotConfig cfg_;
  Go2DdsInterface io_;
  StateEstimator est_;
  LocomotionController loco_;
  std::thread thread_;
  std::atomic<bool> running_{false}, fault_{false};
  uint32_t last_tick_ = 0;
  double last_tick_change_ = 0.0;
  std::atomic<unsigned long> cycles_{0};
  std::atomic<double> max_jitter_ms_{0.0};
  std::atomic<unsigned long> late_cycles_{0};
  double last_step_time_ = 0.0;
  mutable std::mutex last_mtx_;
  LowLevelState last_state_;
  bool odo_initialised_ = false;
};

}  // namespace go2
