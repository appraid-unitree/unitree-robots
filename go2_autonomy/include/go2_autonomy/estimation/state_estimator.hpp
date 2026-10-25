// Robot pose / velocity estimation.
//
// Two independent sources are supported:
//
//  1. LEG_ODOMETRY (proprioceptive, hardware-portable): body velocity from
//     stance-leg kinematics  v = -(J(q) dq + w x r_foot), yaw from the IMU
//     quaternion, position integrated in the odometry frame. Uses only
//     rt/lowstate fields, so it works identically on the real Go2 with the
//     built-in sport service disabled. Drifts with foot slip.
//
//  2. SPORT_STATE: rt/sportmodestate position/velocity. In unitree_mujoco this
//     is the simulator's ground-truth IMU-site position (see readme: it is
//     NOT available on the real robot once the sport service is switched off).
//     On hardware this slot is where an external localisation source
//     (LiDAR odometry / SLAM / mocap) is plugged in.
//
// Yaw always comes from the IMU quaternion in rt/lowstate.
#pragma once

#include <array>
#include <mutex>

#include "go2_autonomy/common/math.hpp"
#include "go2_autonomy/robot/robot_types.hpp"

namespace go2 {

enum class PoseSource { LEG_ODOMETRY, SPORT_STATE };

struct ExternalPose {  // e.g. rt/sportmodestate (sim GT) or an external localiser
  double x = 0, y = 0, z = 0;
  double vx = 0, vy = 0, vz = 0;  // world frame
  double stamp = 0;
  bool valid = false;
};

struct EstimatedState {
  Pose2D pose;            // odometry/world frame
  Twist2D vel_body;       // body frame
  double height = 0.0;    // base height above stance feet (kinematic)
  double roll = 0, pitch = 0;
  double stamp = 0.0;
  bool valid = false;
  PoseSource source = PoseSource::LEG_ODOMETRY;
};

class LegOdometry {
 public:
  void reset(const Pose2D& p);
  // stance: which feet are believed to be on the ground.
  void update(const LowLevelState& s, const std::array<bool, kNumLegs>& stance, double dt);
  Pose2D pose() const { return pose_; }
  Twist2D velocityBody() const { return v_body_; }
  double height() const { return height_; }

 private:
  Pose2D pose_;
  Twist2D v_body_;
  double height_ = 0.3;
  bool init_ = false;
  double yaw_offset_ = 0.0;
};

class StateEstimator {
 public:
  explicit StateEstimator(PoseSource src = PoseSource::SPORT_STATE) : source_(src) {}

  // External pose older than this is treated as invalid (SPORT_STATE source).
  void setExternalTimeout(double s) { std::lock_guard<std::mutex> l(m_); ext_timeout_ = s; }

  void setSource(PoseSource s) { std::lock_guard<std::mutex> l(m_); source_ = s; }
  PoseSource source() const { std::lock_guard<std::mutex> l(m_); return source_; }

  void resetOdometry(const Pose2D& p) { std::lock_guard<std::mutex> l(m_); odo_.reset(p); }

  // Real-time thread.
  void updateProprioceptive(const LowLevelState& s, const std::array<bool, kNumLegs>& stance, double dt);
  // Any thread (DDS callback).
  void updateExternal(const ExternalPose& p);

  EstimatedState estimate() const;
  EstimatedState legOdometryEstimate() const;  // for comparison / logging
  ExternalPose external() const { std::lock_guard<std::mutex> l(m_); return ext_; }

 private:
  mutable std::mutex m_;
  PoseSource source_;
  LegOdometry odo_;
  ExternalPose ext_;
  double yaw_ = 0.0, roll_ = 0.0, pitch_ = 0.0, gyro_z_ = 0.0;
  double stamp_ = 0.0;
  bool have_state_ = false;
  double ext_timeout_ = 0.3;
};

}  // namespace go2
