#include "go2_autonomy/estimation/state_estimator.hpp"

#include <cmath>

#include "go2_autonomy/robot/go2_kinematics.hpp"

namespace go2 {

void LegOdometry::reset(const Pose2D& p) {
  pose_ = p;
  v_body_ = {};
  init_ = false;
}

void LegOdometry::update(const LowLevelState& s, const std::array<bool, kNumLegs>& stance, double dt) {
  const Mat3 R = quatToMat(s.imu_quat[0], s.imu_quat[1], s.imu_quat[2], s.imu_quat[3]);
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  if (!init_) {
    yaw_offset_ = pose_.yaw - yaw;  // keep the reset yaw as odometry origin
    init_ = true;
  }
  const Vec3 w{s.imu_gyro[0], s.imu_gyro[1], s.imu_gyro[2]};
  // Rotation body -> yaw-aligned (levelled) frame: remove roll and pitch.
  const Mat3 Rz = rotZ(yaw);
  const Mat3 Rlevel = [&] {
    Mat3 out;
    const Mat3 RzT = Rz.transpose();
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) {
        double acc = 0;
        for (int k = 0; k < 3; ++k) acc += RzT(r, k) * R(k, c);
        out(r, c) = acc;
      }
    return out;
  }();

  Vec3 v_sum{0, 0, 0};
  double h_sum = 0;
  int n = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (!stance[leg]) continue;
    LegJoints q{s.q[motorIndex(leg, 0)], s.q[motorIndex(leg, 1)], s.q[motorIndex(leg, 2)]};
    Vec3 dq{s.dq[motorIndex(leg, 0)], s.dq[motorIndex(leg, 1)], s.dq[motorIndex(leg, 2)]};
    const Mat3 J = Go2Kinematics::jacobian(leg, q);
    const Vec3 r = Go2Geometry::hipOffset(leg) + Go2Kinematics::forward(leg, q);
    const Vec3 v_foot_rel = J * dq + w.cross(r);  // foot-centre velocity w.r.t. body, body frame
    // The foot is a sphere (r = 22 mm) that rolls on the ground: its centre
    // moves with w_calf x (r * z_up) even without slip (~7 % of stride).
    const double w_calf_y = w.y + dq.y + dq.z;
    // (A lateral rolling term about x was evaluated and made lateral odometry
    // worse in simulation, so it is intentionally omitted.)
    const Vec3 v_roll{w_calf_y * Go2Geometry::kFootRadius, 0.0, 0.0};
    v_sum += v_roll - v_foot_rel;                  // contact point is static in the world
    h_sum += -(Rlevel * r).z;
    ++n;
  }
  if (n > 0) {
    const Vec3 v_body = v_sum * (1.0 / n);
    const Vec3 v_level = Rlevel * v_body;  // planar velocity in yaw frame
    const double a = clamp(dt / 0.04, 0.0, 1.0);  // ~40 ms low-pass
    v_body_.vx += (v_level.x - v_body_.vx) * a;
    v_body_.vy += (v_level.y - v_body_.vy) * a;
    height_ += (h_sum / n - height_) * a;
  }
  v_body_.wz = w.z;
  pose_.yaw = wrapAngle(yaw + yaw_offset_);
  const double c = std::cos(pose_.yaw), sn = std::sin(pose_.yaw);
  pose_.x += (c * v_body_.vx - sn * v_body_.vy) * dt;
  pose_.y += (sn * v_body_.vx + c * v_body_.vy) * dt;
}

void StateEstimator::updateProprioceptive(const LowLevelState& s, const std::array<bool, kNumLegs>& stance,
                                          double dt) {
  std::lock_guard<std::mutex> l(m_);
  odo_.update(s, stance, dt);
  const Mat3 R = quatToMat(s.imu_quat[0], s.imu_quat[1], s.imu_quat[2], s.imu_quat[3]);
  yaw_ = std::atan2(R(1, 0), R(0, 0));
  roll_ = s.imu_rpy[0];
  pitch_ = s.imu_rpy[1];
  gyro_z_ = s.imu_gyro[2];
  stamp_ = s.receive_time;
  have_state_ = true;
}

void StateEstimator::updateExternal(const ExternalPose& p) {
  std::lock_guard<std::mutex> l(m_);
  ext_ = p;
}

EstimatedState StateEstimator::legOdometryEstimate() const {
  std::lock_guard<std::mutex> l(m_);
  EstimatedState e;
  e.pose = odo_.pose();
  e.vel_body = odo_.velocityBody();
  e.height = odo_.height();
  e.roll = roll_;
  e.pitch = pitch_;
  e.stamp = stamp_;
  e.valid = have_state_;
  e.source = PoseSource::LEG_ODOMETRY;
  return e;
}

EstimatedState StateEstimator::estimate() const {
  if (source() == PoseSource::LEG_ODOMETRY) return legOdometryEstimate();
  std::lock_guard<std::mutex> l(m_);
  EstimatedState e;
  e.source = PoseSource::SPORT_STATE;
  e.pose = {ext_.x, ext_.y, yaw_};
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  e.vel_body = {c * ext_.vx + s * ext_.vy, -s * ext_.vx + c * ext_.vy, gyro_z_};
  e.height = odo_.height();
  e.roll = roll_;
  e.pitch = pitch_;
  e.stamp = stamp_;
  // Stale external pose (topic stopped / simulator paused) => invalid.
  e.valid = have_state_ && ext_.valid && (monotonicSeconds() - ext_.stamp) < ext_timeout_;
  return e;
}

}  // namespace go2
