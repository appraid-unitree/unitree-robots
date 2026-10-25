// Simulated multi-layer range scanner for unitree_mujoco (go2_autonomy extension).
//
// Casts rays from a mount point on base_link against the MuJoCo scene at a
// fixed rate and publishes the hits as a sensor_msgs/PointCloud2 (x, y, z
// float32, base_link frame) over Unitree SDK2 DDS.  Only geometric range data
// relative to the robot is published -- no world pose -- so the consumer has to
// localise itself exactly as it would with a real LiDAR.
//
// Thread safety: rays are cast on a PRIVATE mjData whose kinematics is rebuilt
// from a snapshot of the simulator's qpos / mocap state, so the physics thread
// is never blocked and its mjData is only read (same practice as the stock
// unitree_sdk2_bridge).
#pragma once

#include <mujoco/mujoco.h>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

struct RangeScanConfig {
  std::string topic = "rt/sim/range_cloud";
  std::string mount_body = "base_link";
  double mount_pos[3] = {0.0, 0.0, 0.10};       // on top of the trunk
  std::vector<double> elevations_deg = {0.0, -10.0, -20.0};
  double azimuth_step_deg = 2.0;                // 180 beams per layer
  double min_range = 0.05, max_range = 6.0;
  double noise_std = 0.01;                      // [m]
  int period_us = 100000;                       // 10 Hz
};

class RangeScanPublisher {
 public:
  RangeScanPublisher(mjModel* m, mjData* d, RangeScanConfig cfg = RangeScanConfig())
      : m_(m), d_(d), cfg_(std::move(cfg)), rng_(42), noise_(0.0, cfg_.noise_std) {
    body_ = mj_name2id(m_, mjOBJ_BODY, cfg_.mount_body.c_str());
    if (body_ < 0) {
      std::cerr << "[range_scan] mount body '" << cfg_.mount_body << "' not found, scanner disabled" << std::endl;
      return;
    }
    scratch_ = mj_makeData(m_);
    for (double el : cfg_.elevations_deg) {
      const int n = (int)std::lround(360.0 / cfg_.azimuth_step_deg);
      for (int k = 0; k < n; ++k) {
        const double az = (-180.0 + k * cfg_.azimuth_step_deg) * M_PI / 180.0, e = el * M_PI / 180.0;
        dirs_.push_back({std::cos(e) * std::cos(az), std::cos(e) * std::sin(az), std::sin(e)});
      }
    }
    // Ignore visual-only meshes (group 2): the scanner sees collision geometry.
    for (int g = 0; g < mjNGROUP; ++g) geomgroup_[g] = (g == 2) ? 0 : 1;
    publisher_ = std::make_shared<unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_>>(cfg_.topic);
    publisher_->InitChannel();
    std::cout << "[range_scan] publishing " << dirs_.size() << " beams on " << cfg_.topic << " at "
              << 1e6 / cfg_.period_us << " Hz" << std::endl;
  }

  ~RangeScanPublisher() {
    thread_.reset();
    if (scratch_) mj_deleteData(scratch_);
  }

  void start() {
    if (body_ < 0) return;
    thread_ = std::make_shared<unitree::common::RecurrentThread>("range_scan", UT_CPU_ID_NONE, cfg_.period_us,
                                                                 [this]() { this->scanOnce(); });
  }

 private:
  void scanOnce() {
    // Snapshot generalized coordinates (and mocap bodies) and rebuild kinematics privately.
    std::memcpy(scratch_->qpos, d_->qpos, sizeof(mjtNum) * m_->nq);
    if (m_->nmocap) {
      std::memcpy(scratch_->mocap_pos, d_->mocap_pos, sizeof(mjtNum) * 3 * m_->nmocap);
      std::memcpy(scratch_->mocap_quat, d_->mocap_quat, sizeof(mjtNum) * 4 * m_->nmocap);
    }
    const double sim_time = d_->time;
    mj_kinematics(m_, scratch_);

    const mjtNum* R = scratch_->xmat + 9 * body_;  // body -> world
    const mjtNum* p = scratch_->xpos + 3 * body_;
    mjtNum origin[3];
    for (int r = 0; r < 3; ++r)
      origin[r] = p[r] + R[3 * r] * cfg_.mount_pos[0] + R[3 * r + 1] * cfg_.mount_pos[1] + R[3 * r + 2] * cfg_.mount_pos[2];

    std::vector<float> pts;
    pts.reserve(dirs_.size() * 3);
    for (const auto& db : dirs_) {
      mjtNum dw[3];
      for (int r = 0; r < 3; ++r) dw[r] = R[3 * r] * db[0] + R[3 * r + 1] * db[1] + R[3 * r + 2] * db[2];
      int geomid = -1;
      mjtNum dist = mj_ray(m_, scratch_, origin, dw, geomgroup_, 1, body_, &geomid);
      if (dist < 0 || geomid < 0) continue;
      dist += noise_(rng_);
      if (dist < cfg_.min_range || dist > cfg_.max_range) continue;
      // Point in base_link frame.
      pts.push_back((float)(cfg_.mount_pos[0] + db[0] * dist));
      pts.push_back((float)(cfg_.mount_pos[1] + db[1] * dist));
      pts.push_back((float)(cfg_.mount_pos[2] + db[2] * dist));
    }

    sensor_msgs::msg::dds_::PointCloud2_ msg;
    const double sec = std::floor(sim_time);
    msg.header().stamp().sec((int32_t)sec);
    msg.header().stamp().nanosec((uint32_t)((sim_time - sec) * 1e9));
    msg.header().frame_id("base_link");
    msg.height(1);
    msg.width((uint32_t)(pts.size() / 3));
    std::vector<sensor_msgs::msg::dds_::PointField_> fields(3);
    const char* names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      fields[i].name(names[i]);
      fields[i].offset(4 * i);
      fields[i].datatype(7);  // FLOAT32
      fields[i].count(1);
    }
    msg.fields(fields);
    msg.is_bigendian(false);
    msg.point_step(12);
    msg.row_step(12 * msg.width());
    std::vector<uint8_t> data(pts.size() * sizeof(float));
    if (!pts.empty()) std::memcpy(data.data(), pts.data(), data.size());
    msg.data(data);
    msg.is_dense(true);
    publisher_->Write(msg);
  }

  mjModel* m_;
  mjData* d_;
  mjData* scratch_ = nullptr;
  RangeScanConfig cfg_;
  int body_ = -1;
  std::vector<std::array<double, 3>> dirs_;
  mjtByte geomgroup_[mjNGROUP];
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::shared_ptr<unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_>> publisher_;
  unitree::common::RecurrentThreadPtr thread_;
};
