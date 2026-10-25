#pragma once

#include <vector>

#include "go2_autonomy/common/math.hpp"

namespace go2 {

// Range points in the robot body (base_link) frame.
struct PointCloud {
  std::vector<Vec3> points;
  double stamp = 0.0;         // sensor time [s] (simulation time in MuJoCo)
  double receive_time = 0.0;  // host monotonic time [s]
  unsigned long seq = 0;
};

}  // namespace go2
