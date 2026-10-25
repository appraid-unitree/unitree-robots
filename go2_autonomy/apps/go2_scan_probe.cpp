// Diagnostic: report which DDS feedback channels are alive and at what rate.
// usage: go2_scan_probe [network_interface] [domain_id] [seconds]
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "go2_autonomy/comm/go2_dds_interface.hpp"

using namespace go2;

int main(int argc, char** argv) {
  DdsConfig cfg;
  if (argc > 1) cfg.network_interface = argv[1];
  if (argc > 2) cfg.domain_id = std::atoi(argv[2]);
  const double T = argc > 3 ? std::atof(argv[3]) : 3.0;
  Go2DdsInterface io(cfg);
  io.init();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const unsigned long ls0 = io.lowstateCount(), sc0 = io.scanCount();
  const LowLevelState s0 = io.readState();
  const double t0 = monotonicSeconds();
  std::this_thread::sleep_for(std::chrono::duration<double>(T));
  const double dt = monotonicSeconds() - t0;
  const LowLevelState s = io.readState();
  const ExternalPose sp = io.sportState();
  PointCloud pc;
  const bool have_scan = io.latestScan(pc, 0);

  std::printf("rt/lowstate      : %6.1f Hz   (sim tick advanced %.3f s in %.3f s wall -> real-time factor %.2f)\n",
              (io.lowstateCount() - ls0) / dt, (s.tick - s0.tick) * 1e-3, dt, (s.tick - s0.tick) * 1e-3 / dt);
  std::printf("  q[FR]=(%.3f %.3f %.3f) rpy=(%.3f %.3f %.3f) gyro_z=%.3f\n", s.q[0], s.q[1], s.q[2], s.imu_rpy[0],
              s.imu_rpy[1], s.imu_rpy[2], s.imu_gyro[2]);
  std::printf("rt/sportmodestate: %s  pos=(%.3f %.3f %.3f)\n", sp.valid ? "alive" : "MISSING", sp.x, sp.y, sp.z);
  std::printf("%s: %6.1f Hz   points=%zu\n", cfg.scan_topic.c_str(), (io.scanCount() - sc0) / dt,
              have_scan ? pc.points.size() : (size_t)0);
  if (have_scan) {
    double minr = 1e9;
    for (auto& p : pc.points) minr = std::min(minr, std::hypot(p.x, p.y));
    std::printf("  closest planar return: %.3f m\n", minr);
  }
  std::fflush(stdout);
  return (io.lowstateCount() > ls0) ? 0 : 1;
}
