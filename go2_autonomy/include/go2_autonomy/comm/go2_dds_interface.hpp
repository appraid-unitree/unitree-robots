// Unitree SDK2 / DDS communication layer.
//
//   publish   rt/lowcmd          unitree_go::LowCmd_        (500 Hz, from the control loop)
//   subscribe rt/lowstate        unitree_go::LowState_      motors + IMU
//   subscribe rt/sportmodestate  unitree_go::SportModeState_ position/velocity (sim ground truth;
//                                not published by a real Go2 in low-level mode)
//   subscribe <scan topic>       sensor_msgs::PointCloud2_  range points, base_link frame
//
// The same class is used against unitree_mujoco and against the real robot;
// only the network interface / domain id differ (sim: domain 1 on "lo",
// robot: domain 0 on the Ethernet interface).
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "go2_autonomy/estimation/state_estimator.hpp"
#include "go2_autonomy/perception/point_cloud.hpp"
#include "go2_autonomy/robot/robot_types.hpp"

namespace go2 {

struct DdsConfig {
  int domain_id = 1;                 // unitree_mujoco default (real robot: 0)
  std::string network_interface = "lo";
  std::string lowcmd_topic = "rt/lowcmd";
  std::string lowstate_topic = "rt/lowstate";
  std::string sportstate_topic = "rt/sportmodestate";
  std::string scan_topic = "rt/sim/range_cloud";
  bool subscribe_sportstate = true;
  bool subscribe_scan = true;
};

class Go2DdsInterface : public LowLevelIO {
 public:
  explicit Go2DdsInterface(const DdsConfig& cfg);
  ~Go2DdsInterface() override;

  // Initialises the DDS participant (ChannelFactory) once per process.
  void init();

  // LowLevelIO
  LowLevelState readState() override;
  void writeCommand(const JointCommand& cmd) override;

  ExternalPose sportState() const;
  bool latestScan(PointCloud& out, unsigned long newer_than_seq) const;

  double lowstateAge() const;  // seconds since last rt/lowstate
  unsigned long lowstateCount() const;
  unsigned long lowcmdCount() const;
  unsigned long scanCount() const;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;  // shared with the DDS callbacks (see destructor)
  DdsConfig cfg_;
};

}  // namespace go2
