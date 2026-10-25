#include "go2_autonomy/locomotion/locomotion_controller.hpp"

#include <cmath>

namespace go2 {

namespace {
constexpr double kGravity = 9.81;
// Trot: FR+RL in phase, FL+RR half a cycle later.
constexpr double kPhaseOffset[kNumLegs] = {0.0, 0.5, 0.5, 0.0};

double cosineBlend(double s) { s = clamp(s, 0.0, 1.0); return 0.5 - 0.5 * std::cos(kPi * s); }
double cycloid(double s) { s = clamp(s, 0.0, 1.0); return s - std::sin(2.0 * kPi * s) / (2.0 * kPi); }
}  // namespace

const char* toString(LocoMode m) {
  switch (m) {
    case LocoMode::PASSIVE: return "PASSIVE";
    case LocoMode::STANDING_UP: return "STANDING_UP";
    case LocoMode::STAND: return "STAND";
    case LocoMode::WALK: return "WALK";
    case LocoMode::SITTING_DOWN: return "SITTING_DOWN";
    case LocoMode::DAMPING: return "DAMPING";
  }
  return "?";
}

LocomotionController::LocomotionController(const GaitParams& params) : p_(params) {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    LegJoints qc, qs;
    Go2Kinematics::inverse(leg, {p_.foot_x_offset, Go2Geometry::sideSign(leg) * Go2Geometry::kAbadLink,
                                 -p_.crouch_height}, qc);
    Go2Kinematics::inverse(leg, nominalFoot(leg), qs);
    for (int j = 0; j < 3; ++j) {
      crouch_q_[motorIndex(leg, j)] = qc[j];
      stand_q_[motorIndex(leg, j)] = qs[j];
    }
    foot_cmd_[leg] = nominalFoot(leg);
    stance_[leg] = true;
  }
}

// ---------------------------------------------------------------- requests
void LocomotionController::requestStandUp() { std::lock_guard<std::mutex> l(req_mtx_); req_standup_ = true; }
void LocomotionController::requestSitDown() {
  std::lock_guard<std::mutex> l(req_mtx_);
  req_sit_ = true;
  req_walk_ = false;
  cmd_target_ = Twist2D{};
}
void LocomotionController::requestDamping() {
  // Damping also cancels any pending walk / velocity request so the robot can
  // never resume walking on its own after an e-stop or a fault.
  std::lock_guard<std::mutex> l(req_mtx_);
  req_damp_ = true;
  req_walk_ = false;
  req_standup_ = false;
  cmd_target_ = Twist2D{};
}
void LocomotionController::requestWalk(bool walk) { std::lock_guard<std::mutex> l(req_mtx_); req_walk_ = walk; }

void LocomotionController::setVelocityCommand(const Twist2D& c) {
  std::lock_guard<std::mutex> l(req_mtx_);
  cmd_target_.vx = clamp(c.vx, -p_.max_vx, p_.max_vx);
  cmd_target_.vy = clamp(c.vy, -p_.max_vy, p_.max_vy);
  cmd_target_.wz = clamp(c.wz, -p_.max_wz, p_.max_wz);
}

void LocomotionController::setMeasuredBodyVelocity(const Twist2D& v, bool valid) {
  std::lock_guard<std::mutex> l(req_mtx_);
  v_meas_ = v;
  v_meas_valid_ = valid;
}

bool LocomotionController::isReady() const {
  const LocoMode m = mode_.load();
  return (m == LocoMode::STAND && stand_complete_.load()) || m == LocoMode::WALK;
}

LocoTelemetry LocomotionController::telemetry() const {
  std::lock_guard<std::mutex> l(tel_mtx_);
  return tel_;
}

// ---------------------------------------------------------------- helpers
Vec3 LocomotionController::nominalFoot(int leg) const {
  const double side = Go2Geometry::sideSign(leg);
  return {p_.foot_x_offset, side * (Go2Geometry::kAbadLink + p_.foot_y_extra), -p_.body_height};
}

void LocomotionController::poseFromFootTargets(const std::array<Vec3, kNumLegs>& feet,
                                               std::array<LegJoints, kNumLegs>& q, bool& saturated) const {
  saturated = false;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (!Go2Kinematics::inverse(leg, feet[leg], q[leg])) saturated = true;
    Go2Kinematics::clampToLimits(leg, q[leg]);
  }
}

void LocomotionController::levellingOffsets(const LowLevelState& s, std::array<double, kNumLegs>& dz) const {
  // Positive pitch = nose down -> extend front legs. Positive roll = left side
  // up -> shorten left legs. Small-angle kinematic levelling.
  const double roll = s.imu_rpy[0], pitch = s.imu_rpy[1];
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 h = Go2Geometry::hipOffset(leg);
    dz[leg] = p_.att_gain * (-h.x * std::tan(pitch) + h.y * std::tan(roll));
    dz[leg] = clamp(dz[leg], -0.05, 0.05);
  }
}

void LocomotionController::fillLeg(JointCommand& c, int leg, const LegJoints& q, double kp, double kd,
                                   const Vec3& foot_vel, const LegJoints& q_meas, bool stance,
                                   int n_stance) const {
  // Joint velocity feed-forward from the desired foot velocity.
  const Mat3 J = Go2Kinematics::jacobian(leg, q);
  // Solve J * dq = v with Cramer's rule (3x3).
  const double det = J(0, 0) * (J(1, 1) * J(2, 2) - J(1, 2) * J(2, 1)) -
                     J(0, 1) * (J(1, 0) * J(2, 2) - J(1, 2) * J(2, 0)) +
                     J(0, 2) * (J(1, 0) * J(2, 1) - J(1, 1) * J(2, 0));
  LegJoints dq{0, 0, 0};
  if (std::fabs(det) > 1e-6) {
    const double v[3] = {foot_vel.x, foot_vel.y, foot_vel.z};
    for (int col = 0; col < 3; ++col) {
      Mat3 Jc = J;
      for (int r = 0; r < 3; ++r) Jc(r, col) = v[r];
      const double dc = Jc(0, 0) * (Jc(1, 1) * Jc(2, 2) - Jc(1, 2) * Jc(2, 1)) -
                        Jc(0, 1) * (Jc(1, 0) * Jc(2, 2) - Jc(1, 2) * Jc(2, 0)) +
                        Jc(0, 2) * (Jc(1, 0) * Jc(2, 1) - Jc(1, 1) * Jc(2, 0));
      dq[col] = clamp(dc / det, -15.0, 15.0);
    }
  }
  LegJoints tau{0, 0, 0};
  if (stance && p_.gravity_ff && n_stance > 0) {
    const Mat3 Jm = Go2Kinematics::jacobian(leg, q_meas);
    const Vec3 f{0, 0, -p_.total_mass * kGravity / n_stance};  // force the foot applies to the ground
    const Vec3 t = Jm.transpose() * f;
    tau = {t.x, t.y, t.z};
  }
  const double tau_max[3] = {Go2Geometry::kHipTauMax, Go2Geometry::kThighTauMax, Go2Geometry::kCalfTauMax};
  for (int j = 0; j < 3; ++j) {
    const int i = motorIndex(leg, j);
    c.q[i] = q[j];
    c.dq[i] = dq[j];
    c.kp[i] = kp;
    c.kd[i] = kd;
    c.tau[i] = clamp(tau[j], -0.5 * tau_max[j], 0.5 * tau_max[j]);
  }
}

bool LocomotionController::jointsSettled(const LowLevelState& s, double dt) {
  // Same criterion as stand_go2_3step_ik_pd_stable.cpp: every 100 ms compare
  // encoder positions; two consecutive windows with < 0.01 rad change.
  settle_timer_ += dt;
  if (settle_timer_ < 0.1) return false;
  settle_timer_ = 0.0;
  bool stable = true;
  for (int i = 0; i < kNumMotors; ++i) {
    if (std::fabs(s.q[i] - settle_ref_q_[i]) > 0.01) stable = false;
    settle_ref_q_[i] = s.q[i];
  }
  settle_windows_ = stable ? settle_windows_ + 1 : 0;
  return settle_windows_ >= 2;
}

// ---------------------------------------------------------------- main loop
JointCommand LocomotionController::update(const LowLevelState& s, double dt) {
  bool standup, sit, damp, walk;
  {
    std::lock_guard<std::mutex> l(req_mtx_);
    standup = req_standup_; sit = req_sit_; damp = req_damp_; walk = req_walk_;
    req_standup_ = req_sit_ = req_damp_ = false;
  }

  if (!s.valid) return computePassive(s);

  LocoMode m = mode_.load();
  if (damp) {
    mode_.store(LocoMode::DAMPING);
    stand_complete_.store(false);
    m = LocoMode::DAMPING;
    cmd_ = Twist2D{};
  } else if (standup && (m == LocoMode::PASSIVE || m == LocoMode::DAMPING)) {
    enterStandUp(s);
    m = mode_.load();
  }
  if (sit && (m == LocoMode::STAND)) { enterSitDown(s); m = mode_.load(); }
  if (m == LocoMode::STAND && stand_complete_.load() && walk) {
    mode_.store(LocoMode::WALK);
    m = LocoMode::WALK;
    phase_ = 0.0;
    stopping_ = false;
    cmd_ = Twist2D{};  // always accelerate from rest (slew-limited)
    wz_integral_ = vx_integral_ = vy_integral_ = 0.0;
    gyro_z_filt_ = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      // Continue from the feet's CURRENT commanded positions (the stand blend
      // may still be in progress) so the joint targets never jump.
      stance_[leg] = true;
      foot_cmd_[leg].z = -p_.body_height;
    }
  }
  if (m == LocoMode::WALK) stopping_ = !walk;

  JointCommand out;
  switch (m) {
    case LocoMode::PASSIVE: out = computePassive(s); break;
    case LocoMode::DAMPING: out = computeDamping(); break;
    case LocoMode::STANDING_UP: out = computeStandUp(s, dt); break;
    case LocoMode::SITTING_DOWN: out = computeSitDown(s, dt); break;
    case LocoMode::STAND: out = computeStand(s, dt); break;
    case LocoMode::WALK: out = computeWalk(s, dt); break;
  }
  return out;
}

JointCommand LocomotionController::computePassive(const LowLevelState&) const {
  JointCommand c;
  for (int i = 0; i < kNumMotors; ++i) { c.kp[i] = 0; c.kd[i] = 0.5; }
  return c;
}

JointCommand LocomotionController::computeDamping() const {
  JointCommand c;
  for (int i = 0; i < kNumMotors; ++i) { c.kp[i] = 0; c.kd[i] = 3.0; }
  return c;
}

void LocomotionController::enterStandUp(const LowLevelState& s) {
  seq_start_q_ = s.q;
  seq_time_ = 0.0;
  seq_stage_ = 0;
  settle_timer_ = 0.0;
  settle_windows_ = 0;
  settle_ref_q_ = s.q;
  stand_complete_.store(false);
  mode_.store(LocoMode::STANDING_UP);
}

void LocomotionController::enterSitDown(const LowLevelState& s) {
  seq_start_q_ = s.q;
  seq_time_ = 0.0;
  stand_complete_.store(false);
  mode_.store(LocoMode::SITTING_DOWN);
}

JointCommand LocomotionController::computeStandUp(const LowLevelState& s, double dt) {
  seq_time_ += dt;
  JointCommand c;
  std::array<double, kNumMotors> q_des{};
  if (seq_stage_ == 0) {  // measured -> crouch
    const double a = cosineBlend(seq_time_ / p_.standup_crouch_time);
    for (int i = 0; i < kNumMotors; ++i) q_des[i] = seq_start_q_[i] + a * (crouch_q_[i] - seq_start_q_[i]);
    if (seq_time_ >= p_.standup_crouch_time) { seq_stage_ = 1; seq_time_ = 0.0; }
  } else if (seq_stage_ == 1) {  // crouch -> stand
    const double a = cosineBlend(seq_time_ / p_.standup_rise_time);
    for (int i = 0; i < kNumMotors; ++i) q_des[i] = crouch_q_[i] + a * (stand_q_[i] - crouch_q_[i]);
    if (seq_time_ >= p_.standup_rise_time) { seq_stage_ = 2; seq_time_ = 0.0; settle_windows_ = 0; }
  } else {  // hold until encoders show the motion has stopped
    q_des = stand_q_;
    if (jointsSettled(s, dt)) {
      stand_complete_.store(true);
      stand_blend_ = 1.0;
      for (int leg = 0; leg < kNumLegs; ++leg) { foot_cmd_[leg] = nominalFoot(leg); stand_from_[leg] = foot_cmd_[leg]; }
      mode_.store(LocoMode::STAND);
    }
  }
  for (int i = 0; i < kNumMotors; ++i) {
    c.q[i] = q_des[i];
    c.kp[i] = p_.kp_stand;
    c.kd[i] = p_.kd_stand;
  }
  return c;
}

JointCommand LocomotionController::computeSitDown(const LowLevelState&, double dt) {
  seq_time_ += dt;
  JointCommand c;
  const double a = cosineBlend(seq_time_ / p_.sit_time);
  for (int i = 0; i < kNumMotors; ++i) {
    c.q[i] = seq_start_q_[i] + a * (crouch_q_[i] - seq_start_q_[i]);
    c.kp[i] = p_.kp_stand;
    c.kd[i] = p_.kd_stand;
  }
  if (seq_time_ > p_.sit_time + 0.5) mode_.store(LocoMode::DAMPING);
  return c;
}

JointCommand LocomotionController::computeStand(const LowLevelState& s, double dt) {
  std::array<Vec3, kNumLegs> feet;
  std::array<double, kNumLegs> dz{};
  levellingOffsets(s, dz);
  stand_blend_ = std::min(1.0, stand_blend_ + dt / 0.4);
  const double a = cosineBlend(stand_blend_);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 nom = nominalFoot(leg);
    feet[leg] = stand_from_[leg] + (nom - stand_from_[leg]) * a;
    feet[leg].z += dz[leg] * 0.5;
    foot_cmd_[leg] = feet[leg];
  }
  std::array<LegJoints, kNumLegs> q;
  bool sat;
  poseFromFootTargets(feet, q, sat);
  JointCommand c;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    LegJoints qm{s.q[motorIndex(leg, 0)], s.q[motorIndex(leg, 1)], s.q[motorIndex(leg, 2)]};
    fillLeg(c, leg, q[leg], p_.kp_stand, p_.kd_stand, {0, 0, 0}, qm, true, 4);
  }
  {
    std::lock_guard<std::mutex> l(tel_mtx_);
    tel_.mode = LocoMode::STAND;
    tel_.cmd_filtered = {};
    tel_.stance = {true, true, true, true};
    tel_.foot_des = feet;
    tel_.roll = s.imu_rpy[0];
    tel_.pitch = s.imu_rpy[1];
    tel_.ik_saturated = sat;
  }
  return c;
}

JointCommand LocomotionController::computeWalk(const LowLevelState& s, double dt) {
  Twist2D target, vmeas;
  bool vmeas_valid;
  {
    std::lock_guard<std::mutex> l(req_mtx_);
    target = cmd_target_;
    vmeas = v_meas_;
    vmeas_valid = v_meas_valid_;
  }
  if (stopping_) target = Twist2D{};
  cmd_.vx = rateLimit(cmd_.vx, target.vx, p_.acc_lin * dt);
  cmd_.vy = rateLimit(cmd_.vy, target.vy, p_.acc_lin * dt);
  cmd_.wz = rateLimit(cmd_.wz, target.wz, p_.acc_ang * dt);

  // Yaw-rate PI loop on the IMU gyro (available on hardware). Compensates the
  // systematic under-rotation / yaw drift of a purely kinematic trot.
  const double gz = s.imu_gyro[2];
  gyro_z_filt_ += (gz - gyro_z_filt_) * clamp(dt / 0.05, 0.0, 1.0);
  const double wz_err = cmd_.wz - gyro_z_filt_;
  wz_integral_ = clamp(wz_integral_ + p_.wz_ki * wz_err * dt, -0.8, 0.8);
  const double wz_eff = clamp(cmd_.wz + p_.wz_kp * wz_err + wz_integral_, -2.0, 2.0);
  // Forward/lateral velocity integral (only when a body-velocity measurement exists).
  if (vmeas_valid) {
    vx_integral_ = clamp(vx_integral_ + p_.v_ki * (cmd_.vx - vmeas.vx) * dt, -0.3, 0.3);
    vy_integral_ = clamp(vy_integral_ + p_.v_ki * (cmd_.vy - vmeas.vy) * dt, -0.2, 0.2);
  }
  const double vx_eff = cmd_.vx + vx_integral_;
  const double vy_eff = cmd_.vy + vy_integral_;

  const double prev_phase = phase_;
  phase_ += dt / p_.period;
  bool boundary = false;
  if (phase_ >= 1.0) { phase_ -= 1.0; boundary = true; }
  if (prev_phase < 0.5 && phase_ >= 0.5) boundary = true;

  // Stop at a touchdown boundary once the command has decayed.
  const bool cmd_small = std::fabs(cmd_.vx) < 0.02 && std::fabs(cmd_.vy) < 0.02 && std::fabs(cmd_.wz) < 0.05;
  if (stopping_ && cmd_small && boundary) {
    for (int leg = 0; leg < kNumLegs; ++leg) { stand_from_[leg] = foot_cmd_[leg]; stand_from_[leg].z = -p_.body_height; }
    stand_blend_ = 0.0;
    cmd_ = Twist2D{};
    wz_integral_ = vx_integral_ = vy_integral_ = 0.0;
    mode_.store(LocoMode::STAND);
    return computeStand(s, dt);
  }

  const double T_st = p_.duty * p_.period;
  std::array<double, kNumLegs> dz{};
  levellingOffsets(s, dz);

  int n_stance = 0;
  std::array<bool, kNumLegs> st{};
  std::array<double, kNumLegs> leg_phase{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    double ph = phase_ + kPhaseOffset[leg];
    if (ph >= 1.0) ph -= 1.0;
    leg_phase[leg] = ph;
    st[leg] = ph < p_.duty;
    if (st[leg]) ++n_stance;
  }

  std::array<Vec3, kNumLegs> feet;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 hip = Go2Geometry::hipOffset(leg);
    const Vec3 nom = nominalFoot(leg);
    const Vec3 prev = foot_cmd_[leg];
    if (st[leg] && !stance_[leg]) { /* touchdown: keep current position */ }
    if (!st[leg] && stance_[leg]) liftoff_[leg] = foot_cmd_[leg];
    stance_[leg] = st[leg];

    if (st[leg]) {
      // Foot fixed in the world => moves opposite to the body twist.
      const Vec3 r = hip + foot_cmd_[leg];
      const Vec3 vf{-vx_eff + wz_eff * r.y, -vy_eff - wz_eff * r.x, 0.0};
      foot_cmd_[leg].x += vf.x * dt;
      foot_cmd_[leg].y += vf.y * dt;
      foot_cmd_[leg].z = -p_.body_height;
    } else {
      const double sw = (leg_phase[leg] - p_.duty) / (1.0 - p_.duty);
      const Vec3 rn = hip + nom;
      // Raibert heuristic: land half a stance ahead of the neutral point,
      // corrected by the velocity tracking error when a measurement exists.
      Vec3 td = nom;
      td.x += 0.5 * T_st * (vx_eff - wz_eff * rn.y);
      td.y += 0.5 * T_st * (vy_eff + wz_eff * rn.x);
      if (vmeas_valid) {
        td.x += p_.raibert_kv * clamp(vmeas.vx - cmd_.vx, -0.5, 0.5);
        td.y += p_.raibert_kv * clamp(vmeas.vy - cmd_.vy, -0.5, 0.5);
      }
      td.x = clamp(td.x, nom.x - 0.15, nom.x + 0.15);
      td.y = clamp(td.y, nom.y - 0.10, nom.y + 0.10);
      const double a = cycloid(sw);
      foot_cmd_[leg].x = liftoff_[leg].x + (td.x - liftoff_[leg].x) * a;
      foot_cmd_[leg].y = liftoff_[leg].y + (td.y - liftoff_[leg].y) * a;
      foot_cmd_[leg].z = -p_.body_height + p_.step_height * 0.5 * (1.0 - std::cos(2.0 * kPi * sw));
    }
    foot_vel_[leg] = (foot_cmd_[leg] - prev) * (1.0 / dt);
    feet[leg] = foot_cmd_[leg];
    feet[leg].z += dz[leg];
  }

  std::array<LegJoints, kNumLegs> q;
  bool sat;
  poseFromFootTargets(feet, q, sat);
  JointCommand c;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    LegJoints qm{s.q[motorIndex(leg, 0)], s.q[motorIndex(leg, 1)], s.q[motorIndex(leg, 2)]};
    // Ramp the feed-forward in/out over the first/last 15 % of stance.
    const bool stance = st[leg];
    double kp = stance ? p_.kp_stance : p_.kp_swing;
    double kd = stance ? p_.kd_stance : p_.kd_swing;
    fillLeg(c, leg, q[leg], kp, kd, foot_vel_[leg], qm, stance, std::max(n_stance, 1));
    if (stance) {
      const double sp = leg_phase[leg] / p_.duty;
      const double w = clamp(std::min(sp, 1.0 - sp) / 0.15, 0.0, 1.0);
      for (int j = 0; j < 3; ++j) c.tau[motorIndex(leg, j)] *= w;
    }
  }
  {
    std::lock_guard<std::mutex> l(tel_mtx_);
    tel_.mode = LocoMode::WALK;
    tel_.gait_phase = phase_;
    tel_.cmd_filtered = cmd_;
    tel_.stance = st;
    tel_.foot_des = feet;
    tel_.roll = s.imu_rpy[0];
    tel_.pitch = s.imu_rpy[1];
    tel_.ik_saturated = sat;
  }
  return c;
}

}  // namespace go2
