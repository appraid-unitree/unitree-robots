// Quadruped locomotion controller: stand-up / stand / trot / sit-down.
//
// Input : LowLevelState (joint encoders + IMU from rt/lowstate), a body-frame
//         velocity command (vx, vy, wz) and an optional measured body
//         velocity (used for Raibert foot-placement feedback).
// Output: JointCommand (q, dq, kp, kd, tau per motor) for rt/lowcmd.
//
// Approach (all in C++, no external solver):
//   * Foot trajectories in the body frame, converted to joints with analytic IK.
//   * Trot gait (diagonal pairs FR+RL / FL+RR), cycloid swing trajectories.
//   * Stance feet move opposite to the commanded body twist.
//   * Raibert-style touchdown placement with optional velocity feedback.
//   * Attitude levelling from IMU roll/pitch (stance foot height offsets).
//   * Gravity feed-forward torque tau = J^T f on stance legs.
//   * Joint-space PD tracking performed by the motor driver (Kp/Kd in LowCmd).
//
// The stand-up / sit-down sequence is a generalisation of
// stand_go2_3step_ik_pd_stable.cpp: IK-derived crouch/stand poses, cosine
// interpolation from the measured joint state, and an encoder-based
// "movement has stopped" check before a sequence is declared complete.
#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>

#include "go2_autonomy/common/math.hpp"
#include "go2_autonomy/robot/go2_kinematics.hpp"
#include "go2_autonomy/robot/robot_types.hpp"

namespace go2 {

struct GaitParams {
  double body_height = 0.30;        // foot-centre depth below hip joint [m]
  double crouch_height = 0.1457;    // from the user's 3-step controller
  double foot_x_offset = 0.0;       // nominal foot x relative to hip [m]
  double foot_y_extra = 0.0;        // extra lateral spread beyond abduction link [m]
  double period = 0.36;             // full gait cycle [s]
  double duty = 0.5;                // stance fraction
  double step_height = 0.08;        // swing apex [m]
  double raibert_kv = 0.08;         // velocity-error feedback gain [s]
  double max_vx = 0.6, max_vy = 0.3, max_wz = 1.2;
  double acc_lin = 0.8;             // command slew limits [m/s^2]
  double acc_ang = 2.0;             // [rad/s^2]
  double att_gain = 0.6;            // attitude levelling gain (0..1)
  double wz_kp = 0.5, wz_ki = 2.0;  // yaw-rate PI on IMU gyro
  double v_ki = 0.5;                // body-velocity integral (needs a velocity estimate)
  double total_mass = 15.2;         // for gravity feed-forward [kg]
  bool gravity_ff = true;
  // Joint PD gains (applied by the motor driver / simulator bridge).
  double kp_stance = 60.0, kd_stance = 2.0;
  double kp_swing = 45.0, kd_swing = 1.5;
  double kp_stand = 60.0, kd_stand = 3.0;
  // Stand-up sequence timings (seconds).
  double standup_crouch_time = 1.0;
  double standup_rise_time = 1.2;
  double sit_time = 1.5;
};

enum class LocoMode { PASSIVE, STANDING_UP, STAND, WALK, SITTING_DOWN, DAMPING };

const char* toString(LocoMode m);

struct LocoTelemetry {
  LocoMode mode = LocoMode::PASSIVE;
  double gait_phase = 0.0;
  Twist2D cmd_filtered;
  std::array<bool, kNumLegs> stance{};
  std::array<Vec3, kNumLegs> foot_des{};  // rel. hip, body frame
  double roll = 0, pitch = 0;
  bool ik_saturated = false;
};

class LocomotionController {
 public:
  explicit LocomotionController(const GaitParams& params = GaitParams());

  // ---- thread-safe user interface -------------------------------------
  void requestStandUp();
  void requestSitDown();
  void requestDamping();                 // emergency: zero stiffness, damping only
  void setVelocityCommand(const Twist2D& cmd);  // body frame, clamped
  void requestWalk(bool walk);           // true: trot (also in place); false: stop & stand
  void setMeasuredBodyVelocity(const Twist2D& v, bool valid);

  LocoMode mode() const { return mode_.load(); }
  bool isReady() const;                  // standing or walking and stable
  bool standUpComplete() const { return stand_complete_.load(); }
  LocoTelemetry telemetry() const;
  const GaitParams& params() const { return p_; }

  // ---- real-time loop (call at a fixed rate, e.g. 500 Hz) ---------------
  JointCommand update(const LowLevelState& s, double dt);

 private:
  void enterStandUp(const LowLevelState& s);
  void enterSitDown(const LowLevelState& s);
  JointCommand computeStandUp(const LowLevelState& s, double dt);
  JointCommand computeSitDown(const LowLevelState& s, double dt);
  JointCommand computeStand(const LowLevelState& s, double dt);
  JointCommand computeWalk(const LowLevelState& s, double dt);
  JointCommand computeDamping() const;
  JointCommand computePassive(const LowLevelState& s) const;

  Vec3 nominalFoot(int leg) const;  // rel. hip, body frame
  void poseFromFootTargets(const std::array<Vec3, kNumLegs>& feet, std::array<LegJoints, kNumLegs>& q,
                           bool& saturated) const;
  void fillLeg(JointCommand& c, int leg, const LegJoints& q, double kp, double kd, const Vec3& foot_vel,
               const LegJoints& q_meas, bool stance, int n_stance) const;
  bool jointsSettled(const LowLevelState& s, double dt);
  void levellingOffsets(const LowLevelState& s, std::array<double, kNumLegs>& dz) const;

  GaitParams p_;
  std::atomic<LocoMode> mode_{LocoMode::PASSIVE};
  std::atomic<bool> stand_complete_{false};

  // Requests from other threads.
  mutable std::mutex req_mtx_;
  bool req_standup_ = false, req_sit_ = false, req_damp_ = false;
  bool req_walk_ = false;
  Twist2D cmd_target_;
  Twist2D v_meas_;
  bool v_meas_valid_ = false;

  // Real-time state (control thread only).
  Twist2D cmd_;  // slew-limited command
  double seq_time_ = 0.0;
  std::array<double, kNumMotors> seq_start_q_{};
  std::array<double, kNumMotors> crouch_q_{}, stand_q_{};
  int seq_stage_ = 0;
  double settle_timer_ = 0.0;
  std::array<double, kNumMotors> settle_ref_q_{};
  int settle_windows_ = 0;

  double phase_ = 0.0;  // gait phase in [0,1)
  double gyro_z_filt_ = 0.0, wz_integral_ = 0.0, vx_integral_ = 0.0, vy_integral_ = 0.0;
  bool stopping_ = false;
  std::array<Vec3, kNumLegs> foot_cmd_{};      // commanded foot pos rel hip (yaw-level frame)
  std::array<Vec3, kNumLegs> foot_vel_{};
  std::array<Vec3, kNumLegs> liftoff_{};
  std::array<bool, kNumLegs> stance_{};
  std::array<Vec3, kNumLegs> stand_from_{};
  double stand_blend_ = 1.0;

  mutable std::mutex tel_mtx_;
  LocoTelemetry tel_;
};

}  // namespace go2
