// Small, dependency-free math helpers shared by every layer.
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace go2 {

constexpr double kPi = 3.14159265358979323846;

inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

// Wrap an angle to (-pi, pi].
inline double wrapAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double norm() const { return std::sqrt(dot(*this)); }
};

// Row-major 3x3 matrix.
struct Mat3 {
  std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double& operator()(int r, int c) { return m[r * 3 + c]; }
  double operator()(int r, int c) const { return m[r * 3 + c]; }
  Vec3 operator*(const Vec3& v) const {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z};
  }
  Mat3 transpose() const {
    Mat3 t;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) t(r, c) = (*this)(c, r);
    return t;
  }
};

// Quaternion (w, x, y, z) -> rotation matrix (body -> world).
inline Mat3 quatToMat(double w, double x, double y, double z) {
  Mat3 R;
  R(0, 0) = 1 - 2 * (y * y + z * z); R(0, 1) = 2 * (x * y - w * z);     R(0, 2) = 2 * (x * z + w * y);
  R(1, 0) = 2 * (x * y + w * z);     R(1, 1) = 1 - 2 * (x * x + z * z); R(1, 2) = 2 * (y * z - w * x);
  R(2, 0) = 2 * (x * z - w * y);     R(2, 1) = 2 * (y * z + w * x);     R(2, 2) = 1 - 2 * (x * x + y * y);
  return R;
}

inline Mat3 rotZ(double yaw) {
  Mat3 R;
  const double c = std::cos(yaw), s = std::sin(yaw);
  R(0, 0) = c; R(0, 1) = -s; R(1, 0) = s; R(1, 1) = c;
  return R;
}

// Host monotonic time [s].
inline double monotonicSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Planar pose in the world (odometry) frame.
struct Pose2D {
  double x = 0, y = 0, yaw = 0;
};

// Body-frame planar velocity command.
struct Twist2D {
  double vx = 0, vy = 0, wz = 0;
};

// Simple first-order rate limiter.
inline double rateLimit(double current, double target, double max_step) {
  return current + clamp(target - current, -max_step, max_step);
}

}  // namespace go2
