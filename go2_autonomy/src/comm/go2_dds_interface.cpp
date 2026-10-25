#include "go2_autonomy/comm/go2_dds_interface.hpp"

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace go2 {

using namespace unitree::robot;

namespace {
constexpr double kPosStop = 2.146e9;
constexpr double kVelStop = 16000.0;

// CRC used by the Go2 motor controller (same as stand_go2_3step_ik_pd_stable.cpp).
uint32_t crc32Core(const uint32_t* ptr, uint32_t len) {
  uint32_t crc = 0xFFFFFFFF;
  const uint32_t poly = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    uint32_t xbit = 1u << 31;
    const uint32_t data = ptr[i];
    for (int bits = 0; bits < 32; bits++) {
      if (crc & 0x80000000) { crc <<= 1; crc ^= poly; } else { crc <<= 1; }
      if (data & xbit) crc ^= poly;
      xbit >>= 1;
    }
  }
  return crc;
}
}  // namespace

struct Go2DdsInterface::Impl {
  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_pub;
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub;
  ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_sub;
  ChannelSubscriberPtr<sensor_msgs::msg::dds_::PointCloud2_> scan_sub;

  unitree_go::msg::dds_::LowCmd_ lowcmd{};
  std::mutex cmd_mtx;

  mutable std::mutex state_mtx;
  LowLevelState state;
  double state_time = 0.0;

  mutable std::mutex sport_mtx;
  ExternalPose sport;

  mutable std::mutex scan_mtx;
  PointCloud scan;

  std::atomic<bool> closed{false};
  std::atomic<unsigned long> lowstate_count{0}, lowcmd_count{0}, scan_count{0};
};

Go2DdsInterface::Go2DdsInterface(const DdsConfig& cfg) : impl_(new Impl), cfg_(cfg) {}

Go2DdsInterface::~Go2DdsInterface() {
  // Destroying a CycloneDDS-C++ DataReader while one of its listener callbacks
  // is in flight trips an assertion inside the SDK's libddscxx
  // (EntityDelegate::prevent_callbacks) -- a race that is hit regularly with a
  // 1 kHz rt/lowstate stream. We therefore never destroy the readers: the
  // callbacks are disabled via `closed` and only capture the shared Impl, which
  // is intentionally kept alive for the rest of the process (one small object
  // per interface instance; the OS reclaims it at exit).
  impl_->closed.store(true);
  static std::vector<std::shared_ptr<Impl>> retired;
  static std::mutex retired_mtx;
  std::lock_guard<std::mutex> l(retired_mtx);
  retired.push_back(impl_);
}

void Go2DdsInterface::init() {
  ChannelFactory::Instance()->Init(cfg_.domain_id, cfg_.network_interface);

  std::shared_ptr<Impl> impl = impl_;  // captured by value in every callback (never `this`)
  auto& c = impl_->lowcmd;
  c.head()[0] = 0xFE;
  c.head()[1] = 0xEF;
  c.level_flag() = 0xFF;
  c.gpio() = 0;
  for (int i = 0; i < 20; i++) {
    c.motor_cmd()[i].mode() = 0x01;  // servo (PMSM) mode
    c.motor_cmd()[i].q() = kPosStop;
    c.motor_cmd()[i].kp() = 0;
    c.motor_cmd()[i].dq() = kVelStop;
    c.motor_cmd()[i].kd() = 0;
    c.motor_cmd()[i].tau() = 0;
  }

  impl_->lowcmd_pub.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(cfg_.lowcmd_topic));
  impl_->lowcmd_pub->InitChannel();

  impl_->lowstate_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(cfg_.lowstate_topic));
  impl_->lowstate_sub->InitChannel(
      [impl](const void* msg) {
        if (impl->closed.load()) return;
        const auto& ls = *static_cast<const unitree_go::msg::dds_::LowState_*>(msg);
        LowLevelState s;
        for (int i = 0; i < kNumMotors; ++i) {
          s.q[i] = ls.motor_state()[i].q();
          s.dq[i] = ls.motor_state()[i].dq();
          s.tau_est[i] = ls.motor_state()[i].tau_est();
        }
        for (int k = 0; k < 4; ++k) s.imu_quat[k] = ls.imu_state().quaternion()[k];
        for (int k = 0; k < 3; ++k) {
          s.imu_rpy[k] = ls.imu_state().rpy()[k];
          s.imu_gyro[k] = ls.imu_state().gyroscope()[k];
          s.imu_acc[k] = ls.imu_state().accelerometer()[k];
        }
        for (int k = 0; k < 4; ++k) s.foot_force[k] = ls.foot_force()[k];
        s.tick = ls.tick();
        s.receive_time = monotonicSeconds();
        s.valid = true;
        {
          std::lock_guard<std::mutex> l(impl->state_mtx);
          impl->state = s;
          impl->state_time = s.receive_time;
        }
        impl->lowstate_count++;
      },
      1);

  if (cfg_.subscribe_sportstate) {
    impl_->sport_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(cfg_.sportstate_topic));
    impl_->sport_sub->InitChannel(
        [impl](const void* msg) {
          if (impl->closed.load()) return;
          const auto& ss = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(msg);
          ExternalPose p;
          p.x = ss.position()[0];
          p.y = ss.position()[1];
          p.z = ss.position()[2];
          p.vx = ss.velocity()[0];
          p.vy = ss.velocity()[1];
          p.vz = ss.velocity()[2];
          p.stamp = monotonicSeconds();
          p.valid = true;
          std::lock_guard<std::mutex> l(impl->sport_mtx);
          impl->sport = p;
        },
        1);
  }

  if (cfg_.subscribe_scan) {
    impl_->scan_sub.reset(new ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>(cfg_.scan_topic));
    impl_->scan_sub->InitChannel(
        [impl](const void* msg) {
          if (impl->closed.load()) return;
          const auto& pc = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(msg);
          int ox = -1, oy = -1, oz = -1;
          for (const auto& f : pc.fields()) {
            if (f.datatype() != 7) continue;  // FLOAT32 only
            if (f.name() == "x") ox = f.offset();
            if (f.name() == "y") oy = f.offset();
            if (f.name() == "z") oz = f.offset();
          }
          if (ox < 0 || oy < 0 || oz < 0 || pc.point_step() == 0) return;
          if ((uint32_t)std::max(ox, std::max(oy, oz)) + 4 > pc.point_step()) return;  // malformed layout
          PointCloud out;
          const size_t n = pc.data().size() / pc.point_step();
          out.points.reserve(n);
          const uint8_t* base = pc.data().data();
          for (size_t i = 0; i < n; ++i) {
            float x, y, z;
            std::memcpy(&x, base + i * pc.point_step() + ox, 4);
            std::memcpy(&y, base + i * pc.point_step() + oy, 4);
            std::memcpy(&z, base + i * pc.point_step() + oz, 4);
            out.points.push_back({x, y, z});
          }
          out.stamp = pc.header().stamp().sec() + 1e-9 * pc.header().stamp().nanosec();
          out.receive_time = monotonicSeconds();
          out.seq = ++impl->scan_count;
          std::lock_guard<std::mutex> l(impl->scan_mtx);
          impl->scan = std::move(out);
        },
        1);
  }
}

LowLevelState Go2DdsInterface::readState() {
  std::lock_guard<std::mutex> l(impl_->state_mtx);
  return impl_->state;
}

void Go2DdsInterface::writeCommand(const JointCommand& cmd) {
  std::lock_guard<std::mutex> l(impl_->cmd_mtx);
  auto& c = impl_->lowcmd;
  for (int i = 0; i < kNumMotors; ++i) {
    c.motor_cmd()[i].mode() = 0x01;
    c.motor_cmd()[i].q() = static_cast<float>(cmd.q[i]);
    c.motor_cmd()[i].dq() = static_cast<float>(cmd.dq[i]);
    c.motor_cmd()[i].kp() = static_cast<float>(cmd.kp[i]);
    c.motor_cmd()[i].kd() = static_cast<float>(cmd.kd[i]);
    c.motor_cmd()[i].tau() = static_cast<float>(cmd.tau[i]);
  }
  c.crc() = crc32Core(reinterpret_cast<uint32_t*>(&c), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  impl_->lowcmd_pub->Write(c);
  impl_->lowcmd_count++;
}

unsigned long Go2DdsInterface::lowstateCount() const { return impl_->lowstate_count.load(); }
unsigned long Go2DdsInterface::lowcmdCount() const { return impl_->lowcmd_count.load(); }
unsigned long Go2DdsInterface::scanCount() const { return impl_->scan_count.load(); }

ExternalPose Go2DdsInterface::sportState() const {
  std::lock_guard<std::mutex> l(impl_->sport_mtx);
  return impl_->sport;
}

bool Go2DdsInterface::latestScan(PointCloud& out, unsigned long newer_than_seq) const {
  std::lock_guard<std::mutex> l(impl_->scan_mtx);
  if (impl_->scan.seq <= newer_than_seq) return false;
  out = impl_->scan;
  return true;
}

double Go2DdsInterface::lowstateAge() const {
  std::lock_guard<std::mutex> l(impl_->state_mtx);
  if (impl_->state_time <= 0) return 1e9;
  return monotonicSeconds() - impl_->state_time;
}

}  // namespace go2
