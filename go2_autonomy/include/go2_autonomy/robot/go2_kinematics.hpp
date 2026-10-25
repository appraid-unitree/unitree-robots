// Go2 leg kinematics (forward, inverse, Jacobian).
//
// Geometry taken from unitree_mujoco/unitree_robots/go2/go2.xml, which matches
// the official Go2 URDF:
//   hip joint  : (+-0.1934, +-0.0465, 0) from base_link origin
//   thigh joint: +-0.0955 lateral from hip joint (abduction axis = x)
//   thigh / calf length: 0.213 m (foot sphere centre, radius 0.022)
//
// Frames: body frame x forward, y left, z up. Foot positions are expressed
// relative to the HIP JOINT of that leg, in the body frame.
#pragma once

#include <array>

#include "go2_autonomy/common/math.hpp"
#include "go2_autonomy/robot/robot_types.hpp"

namespace go2 {

struct Go2Geometry {
  static constexpr double kHipX = 0.1934;
  static constexpr double kHipY = 0.0465;
  static constexpr double kAbadLink = 0.0955;
  static constexpr double kThigh = 0.213;
  static constexpr double kCalf = 0.213;
  static constexpr double kFootRadius = 0.022;

  // Joint limits from go2.xml [rad].
  static constexpr double kHipMin = -1.0472, kHipMax = 1.0472;
  static constexpr double kFrontThighMin = -1.5708, kFrontThighMax = 3.4907;
  static constexpr double kRearThighMin = -0.5236, kRearThighMax = 4.5379;
  static constexpr double kCalfMin = -2.7227, kCalfMax = -0.83776;

  // Torque limits [Nm].
  static constexpr double kHipTauMax = 23.7, kThighTauMax = 23.7, kCalfTauMax = 45.43;

  static double sideSign(int leg) { return (leg == FL || leg == RL) ? 1.0 : -1.0; }
  static double frontSign(int leg) { return (leg == FR || leg == FL) ? 1.0 : -1.0; }
  static Vec3 hipOffset(int leg) { return {frontSign(leg) * kHipX, sideSign(leg) * kHipY, 0.0}; }
};

using LegJoints = std::array<double, 3>;  // hip (abduction), thigh, calf

class Go2Kinematics {
 public:
  // Foot position relative to the hip joint (body frame).
  static Vec3 forward(int leg, const LegJoints& q);

  // Inverse kinematics. Returns false if the target is unreachable; in that
  // case the target is projected to the workspace boundary and the closest
  // solution is still written to q.
  static bool inverse(int leg, const Vec3& foot_rel_hip, LegJoints& q);

  // 3x3 Jacobian d(foot)/d(q) (body frame).
  static Mat3 jacobian(int leg, const LegJoints& q);

  // Clamp joints to the model's position limits.
  static void clampToLimits(int leg, LegJoints& q);
};

}  // namespace go2
