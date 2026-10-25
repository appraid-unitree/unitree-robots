#include "go2_autonomy/app/go2_robot.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace go2 {

Go2Robot::Go2Robot(const RobotConfig& cfg) : cfg_(cfg), io_(cfg.dds), est_(cfg.pose_source), loco_(cfg.gait) {}

Go2Robot::~Go2Robot() { stop(); }

bool Go2Robot::start(double wait_state_timeout_s) {
  io_.init();
  const double t0 = monotonicSeconds();
  while (io_.lowstateCount() == 0) {
    if (monotonicSeconds() - t0 > wait_state_timeout_s) {
      std::cerr << "[Go2Robot] no rt/lowstate received within " << wait_state_timeout_s
                << " s (is the simulator / robot running on domain " << cfg_.dds.domain_id << ", interface "
                << cfg_.dds.network_interface << "?)" << std::endl;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  running_.store(true);
  last_step_time_ = monotonicSeconds();
  last_tick_change_ = last_step_time_;
  thread_ = std::thread(&Go2Robot::controlLoop, this);
  return true;
}

void Go2Robot::controlLoop() {
  // Fixed-rate loop on absolute deadlines (no drift accumulation).
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(cfg_.control_dt));
  auto next = std::chrono::steady_clock::now();
  while (running_.load()) {
    next += period;
    controlStep();
    const auto now = std::chrono::steady_clock::now();
    if (next < now - 10 * period) next = now;  // after a long stall, do not burst to catch up
    std::this_thread::sleep_until(next);
  }
}

void Go2Robot::stop() {
  if (!running_.load()) return;
  // 1) let an ongoing walk come to a stand (gait stops at a touchdown boundary)
  if (loco_.mode() == LocoMode::WALK && !fault_.load()) {
    loco_.setVelocityCommand({});
    loco_.requestWalk(false);
    const double t0 = monotonicSeconds();
    while (loco_.mode() == LocoMode::WALK && monotonicSeconds() - t0 < 3.0)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // 2) damping for a few hundred ms so the LAST command is kp = 0, kd > 0
  if (loco_.mode() != LocoMode::PASSIVE) {
    loco_.requestDamping();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  // 3) terminate and join
  running_.store(false);
  if (thread_.joinable()) thread_.join();
}

LowLevelState Go2Robot::lastState() const {
  std::lock_guard<std::mutex> l(last_mtx_);
  return last_state_;
}

void Go2Robot::controlStep() {
  if (!running_.load()) return;
  const double now = monotonicSeconds();
  const double jitter = std::fabs((now - last_step_time_) - cfg_.control_dt) * 1e3;
  if (cycles_.load() > 100 && jitter > max_jitter_ms_.load()) max_jitter_ms_.store(jitter);
  if (cycles_.load() > 100 && now - last_step_time_ > 2.0 * cfg_.control_dt) late_cycles_++;
  last_step_time_ = now;

  LowLevelState s = io_.readState();
  const double age = io_.lowstateAge();
  // A paused simulator / frozen robot keeps publishing the SAME tick: treat as stale too.
  if (s.valid && s.tick != last_tick_) { last_tick_ = s.tick; last_tick_change_ = now; }
  const double tick_age = now - last_tick_change_;
  const LocoMode m = loco_.mode();
  if ((age > cfg_.lowstate_timeout || tick_age > cfg_.lowstate_timeout) && m != LocoMode::PASSIVE &&
      m != LocoMode::DAMPING) {
    if (!fault_.exchange(true))
      std::cerr << "[Go2Robot] rt/lowstate stale (msg age " << age << " s, tick age " << tick_age
                << " s) -> DAMPING" << std::endl;
    loco_.requestDamping();
  }

  // State estimation (proprioceptive every cycle, external pose when available).
  const LocoTelemetry tel = loco_.telemetry();
  std::array<bool, kNumLegs> stance = tel.stance;
  if (m != LocoMode::WALK) stance = {true, true, true, true};
  if (s.valid) {
    if (!odo_initialised_ && loco_.standUpComplete()) {
      // Anchor the leg-odometry frame to the external pose (if any) once standing.
      const ExternalPose ext = io_.sportState();
      const Mat3 R = quatToMat(s.imu_quat[0], s.imu_quat[1], s.imu_quat[2], s.imu_quat[3]);
      est_.resetOdometry({ext.valid ? ext.x : 0.0, ext.valid ? ext.y : 0.0, std::atan2(R(1, 0), R(0, 0))});
      odo_initialised_ = true;
    }
    est_.updateProprioceptive(s, stance, cfg_.control_dt);
  }
  if (cfg_.dds.subscribe_sportstate) est_.updateExternal(io_.sportState());

  if (cfg_.gait_velocity_feedback && m == LocoMode::WALK) {
    loco_.setMeasuredBodyVelocity(est_.legOdometryEstimate().vel_body, true);
  } else {
    loco_.setMeasuredBodyVelocity({}, false);
  }

  const JointCommand cmd = loco_.update(s, cfg_.control_dt);
  if (s.valid) io_.writeCommand(cmd);
  {
    std::lock_guard<std::mutex> l(last_mtx_);
    last_state_ = s;
  }
  cycles_++;
}

bool Go2Robot::standUp(double timeout_s) {
  if (fault_.load()) {
    std::cerr << "[Go2Robot] standUp refused: communication fault latched" << std::endl;
    return false;
  }
  if (loco_.isReady()) return true;
  loco_.requestStandUp();
  const double t0 = monotonicSeconds();
  while (monotonicSeconds() - t0 < timeout_s) {
    if (fault_.load()) return false;
    if (loco_.mode() == LocoMode::STAND && loco_.standUpComplete()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool Go2Robot::sitDown(double timeout_s) {
  loco_.requestWalk(false);
  const double t0 = monotonicSeconds();
  while (loco_.mode() == LocoMode::WALK && monotonicSeconds() - t0 < timeout_s)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  loco_.requestSitDown();
  while (monotonicSeconds() - t0 < timeout_s) {
    if (loco_.mode() == LocoMode::DAMPING) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace go2
