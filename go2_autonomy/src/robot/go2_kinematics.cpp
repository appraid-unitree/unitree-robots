#include "go2_autonomy/robot/go2_kinematics.hpp"

#include <cmath>

namespace go2 {

Vec3 Go2Kinematics::forward(int leg, const LegJoints& q) {
  const double l1 = Go2Geometry::sideSign(leg) * Go2Geometry::kAbadLink;
  const double l2 = Go2Geometry::kThigh;
  const double l3 = Go2Geometry::kCalf;
  // Sagittal-plane position of the foot relative to the thigh joint.
  const double xs = -l2 * std::sin(q[1]) - l3 * std::sin(q[1] + q[2]);
  const double zs = -l2 * std::cos(q[1]) - l3 * std::cos(q[1] + q[2]);
  // Rotate (xs, l1, zs) about the x axis by the abduction angle.
  const double c = std::cos(q[0]), s = std::sin(q[0]);
  return {xs, l1 * c - zs * s, l1 * s + zs * c};
}

bool Go2Kinematics::inverse(int leg, const Vec3& p, LegJoints& q) {
  const double l1 = Go2Geometry::sideSign(leg) * Go2Geometry::kAbadLink;
  const double l2 = Go2Geometry::kThigh;
  const double l3 = Go2Geometry::kCalf;
  bool ok = true;

  // --- abduction -----------------------------------------------------------
  double r_yz_sq = p.y * p.y + p.z * p.z;
  double zs_sq = r_yz_sq - l1 * l1;
  if (zs_sq < 1e-6) { zs_sq = 1e-6; ok = false; }
  const double zs = -std::sqrt(zs_sq);  // foot is below the thigh joint
  q[0] = std::atan2(p.z, p.y) - std::atan2(zs, l1);
  q[0] = wrapAngle(q[0]);

  // --- sagittal plane ------------------------------------------------------
  const double xs = p.x;
  double d_sq = xs * xs + zs * zs;
  const double d_max = (l2 + l3) * 0.999;
  const double d_min = std::fabs(l2 - l3) + 0.02;
  double d = std::sqrt(d_sq);
  double scale = 1.0;
  if (d > d_max) { scale = d_max / d; ok = false; }
  if (d < d_min) { scale = d_min / std::max(d, 1e-6); ok = false; }
  const double xs2 = xs * scale, zs2 = zs * scale;
  d_sq = xs2 * xs2 + zs2 * zs2;

  double cos_knee = (d_sq - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
  cos_knee = clamp(cos_knee, -1.0, 1.0);
  q[2] = -std::acos(cos_knee);  // Go2 knee is always negative
  q[1] = std::atan2(-xs2, -zs2) - std::atan2(l3 * std::sin(q[2]), l2 + l3 * std::cos(q[2]));
  return ok;
}

Mat3 Go2Kinematics::jacobian(int leg, const LegJoints& q) {
  Mat3 J;
  const double h = 1e-5;
  for (int j = 0; j < 3; ++j) {
    LegJoints qp = q, qm = q;
    qp[j] += h;
    qm[j] -= h;
    const Vec3 d = (forward(leg, qp) - forward(leg, qm)) * (1.0 / (2.0 * h));
    J(0, j) = d.x;
    J(1, j) = d.y;
    J(2, j) = d.z;
  }
  return J;
}

void Go2Kinematics::clampToLimits(int leg, LegJoints& q) {
  const bool front = (leg == FR || leg == FL);
  q[0] = clamp(q[0], Go2Geometry::kHipMin, Go2Geometry::kHipMax);
  q[1] = clamp(q[1], front ? Go2Geometry::kFrontThighMin : Go2Geometry::kRearThighMin,
               front ? Go2Geometry::kFrontThighMax : Go2Geometry::kRearThighMax);
  q[2] = clamp(q[2], Go2Geometry::kCalfMin, Go2Geometry::kCalfMax);
}

}  // namespace go2
