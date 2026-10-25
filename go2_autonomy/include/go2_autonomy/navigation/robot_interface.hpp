// What the navigation layer needs from "a robot". Deliberately small so the
// same Navigator can drive:
//   * Go2Robot  -- our own low-level locomotion over rt/lowcmd (MuJoCo / real Go2)
//   * a future adapter around Unitree's built-in sport-mode controller on the
//     real robot (SportClient::Move(vx, vy, wz)) + its odometry / LiDAR topics.
#pragma once

#include "go2_autonomy/common/math.hpp"
#include "go2_autonomy/estimation/state_estimator.hpp"
#include "go2_autonomy/perception/point_cloud.hpp"

namespace go2 {

class NavRobotInterface {
 public:
  virtual ~NavRobotInterface() = default;
  virtual EstimatedState estimate() const = 0;
  virtual bool latestScan(PointCloud& out, unsigned long newer_than_seq) const = 0;
  virtual void setVelocity(const Twist2D& body_cmd) = 0;  // body frame
  virtual void setWalking(bool walk) = 0;                // false: stop and stand
  virtual bool isReady() const = 0;                      // standing/walking, accepting commands
  virtual bool isWalking() const = 0;
  virtual double footRadius() const { return 0.022; }   // contact sphere -> ground offset
};

}  // namespace go2
