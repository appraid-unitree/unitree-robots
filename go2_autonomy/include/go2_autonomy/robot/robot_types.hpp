// Hardware-neutral robot state / command containers.
//
// Motor order follows the Unitree Go2 SDK convention (identical in the MuJoCo
// model and on the real robot):
//   0..2  FR hip, thigh, calf
//   3..5  FL hip, thigh, calf
//   6..8  RR hip, thigh, calf
//   9..11 RL hip, thigh, calf
#pragma once

#include <array>
#include <cstdint>

namespace go2 {

constexpr int kNumLegs = 4;
constexpr int kNumMotors = 12;

enum Leg : int { FR = 0, FL = 1, RR = 2, RL = 3 };

inline int motorIndex(int leg, int joint) { return leg * 3 + joint; }

// Everything that rt/lowstate actually provides and that we use.
struct LowLevelState {
  std::array<double, kNumMotors> q{};
  std::array<double, kNumMotors> dq{};
  std::array<double, kNumMotors> tau_est{};
  std::array<double, 4> imu_quat{1, 0, 0, 0};  // w x y z  (body -> world)
  std::array<double, 3> imu_rpy{};
  std::array<double, 3> imu_gyro{};            // body frame, rad/s
  std::array<double, 3> imu_acc{};             // body frame, m/s^2
  std::array<int16_t, 4> foot_force{};         // real robot only; zero in MuJoCo
  uint32_t tick = 0;                           // ms tick from the robot / simulator
  double receive_time = 0.0;                   // host monotonic time [s]
  bool valid = false;
};

// One joint-space command for all 12 motors (maps 1:1 to rt/lowcmd).
struct JointCommand {
  std::array<double, kNumMotors> q{};
  std::array<double, kNumMotors> dq{};
  std::array<double, kNumMotors> kp{};
  std::array<double, kNumMotors> kd{};
  std::array<double, kNumMotors> tau{};
};

// Abstract low-level I/O. Implemented by the Unitree SDK2 DDS bridge
// (simulation + real robot) and by an in-process MuJoCo harness used for
// controller unit tests.
class LowLevelIO {
 public:
  virtual ~LowLevelIO() = default;
  virtual LowLevelState readState() = 0;
  virtual void writeCommand(const JointCommand& cmd) = 0;
};

}  // namespace go2
