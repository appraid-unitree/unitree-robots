// Offline development harness: runs the LocomotionController directly against
// the MuJoCo Go2 model (same XML, same PD law as unitree_mujoco's bridge)
// faster than real time. Used only for tuning; final validation always runs
// through the real unitree_mujoco simulator over DDS.
//
// usage: mujoco_harness <scene.xml> "<script>" [log.csv]
//   script: ';' separated commands
//     standup | wait T | walk VX VY WZ T | stop | sit | push FX FY T
//   env GAIT="h=0.30,T=0.36,..." overrides gait parameters; VFB=1 (ground-truth
//   velocity feedback) or VFB=2 (leg-odometry velocity feedback, as on hardware).
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "go2_autonomy/estimation/state_estimator.hpp"
#include "go2_autonomy/locomotion/locomotion_controller.hpp"

using namespace go2;

struct Sim {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  int quat_adr = -1, gyro_adr = -1, acc_adr = -1;
  JointCommand cmd;

  LowLevelState read() const {
    LowLevelState s;
    for (int i = 0; i < 12; ++i) {
      s.q[i] = d->sensordata[i];
      s.dq[i] = d->sensordata[12 + i];
      s.tau_est[i] = d->sensordata[24 + i];
    }
    for (int k = 0; k < 4; ++k) s.imu_quat[k] = d->sensordata[quat_adr + k];
    for (int k = 0; k < 3; ++k) { s.imu_gyro[k] = d->sensordata[gyro_adr + k]; s.imu_acc[k] = d->sensordata[acc_adr + k]; }
    const double w = s.imu_quat[0], x = s.imu_quat[1], y = s.imu_quat[2], z = s.imu_quat[3];
    s.imu_rpy[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    s.imu_rpy[1] = std::asin(clamp(2 * (w * y - z * x), -1.0, 1.0));
    s.imu_rpy[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    s.tick = (uint32_t)std::round(d->time * 1000);
    s.valid = true;
    return s;
  }
  void applyPD() {
    for (int i = 0; i < 12; ++i)
      d->ctrl[i] = cmd.tau[i] + cmd.kp[i] * (cmd.q[i] - d->sensordata[i]) + cmd.kd[i] * (cmd.dq[i] - d->sensordata[12 + i]);
  }
  double yaw() const {
    const double* q = d->qpos + 3;
    return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
  }
};

int main(int argc, char** argv) {
  if (argc < 3) { std::printf("usage: mujoco_harness scene.xml \"script\" [log.csv]\n"); return 1; }
  char err[1000] = "";
  Sim sim;
  sim.m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!sim.m) { std::printf("load error: %s\n", err); return 1; }
  sim.d = mj_makeData(sim.m);
  sim.quat_adr = sim.m->sensor_adr[mj_name2id(sim.m, mjOBJ_SENSOR, "imu_quat")];
  sim.gyro_adr = sim.m->sensor_adr[mj_name2id(sim.m, mjOBJ_SENSOR, "imu_gyro")];
  sim.acc_adr = sim.m->sensor_adr[mj_name2id(sim.m, mjOBJ_SENSOR, "imu_acc")];
  FILE* log = argc > 3 ? std::fopen(argv[3], "w") : nullptr;
  if (log) std::fprintf(log, "t,x,y,z,yaw,roll,pitch,vx_b,vy_b,wz,mode,cmd_vx,cmd_wz\n");

  GaitParams gp;
  if (const char* e = std::getenv("GAIT")) {
    // KEY=VAL,KEY=VAL overrides for tuning
    std::stringstream ss(e);
    std::string kv;
    while (std::getline(ss, kv, ',')) {
      auto eq = kv.find('=');
      if (eq == std::string::npos) continue;
      std::string k = kv.substr(0, eq);
      double v = std::atof(kv.substr(eq + 1).c_str());
      if (k == "h") gp.body_height = v; else if (k == "T") gp.period = v; else if (k == "duty") gp.duty = v;
      else if (k == "sh") gp.step_height = v; else if (k == "kv") gp.raibert_kv = v; else if (k == "att") gp.att_gain = v;
      else if (k == "kps") gp.kp_stance = v; else if (k == "kds") gp.kd_stance = v; else if (k == "kpw") gp.kp_swing = v;
      else if (k == "kdw") gp.kd_swing = v; else if (k == "ff") gp.gravity_ff = v > 0.5; else if (k == "fx") gp.foot_x_offset = v;
      else if (k == "fy") gp.foot_y_extra = v; else if (k == "wzkp") gp.wz_kp = v; else if (k == "wzki") gp.wz_ki = v; else if (k == "vki") gp.v_ki = v;
    }
  }
  LocomotionController loco(gp);
  const int use_vfb = std::getenv("VFB") ? std::atoi(std::getenv("VFB")) : 0;  // 1: GT velocity, 2: leg odometry
  StateEstimator est(PoseSource::LEG_ODOMETRY);


  std::vector<std::string> cmds;
  {
    std::stringstream ss(argv[2]);
    std::string c;
    while (std::getline(ss, c, ';')) cmds.push_back(c);
  }

  const double dt = sim.m->opt.timestep;  // 0.002
  double t = 0;
  double yaw_unwrapped = 0, yaw_prev = 0;
  double max_tilt = 0;
  double min_z = 1e9;
  bool fell = false;
  auto run_for = [&](double T, bool check_ready = false) {
    const double t_end = t + T;
    while (t < t_end) {
      LowLevelState s = sim.read();
      {
        auto tel0 = loco.telemetry();
        std::array<bool, kNumLegs> st = tel0.stance;
        if (loco.mode() != LocoMode::WALK) st = {true, true, true, true};
        est.updateProprioceptive(s, st, dt);
      }
      if (use_vfb == 2 && loco.mode() == LocoMode::WALK) {
        loco.setMeasuredBodyVelocity(est.legOdometryEstimate().vel_body, true);
      } else if (use_vfb == 1) {
        // body-frame velocity from simulator ground truth (sportmodestate equivalent)
        const double yaw = sim.yaw();
        const double vx = sim.d->qvel[0], vy = sim.d->qvel[1];
        Twist2D v{std::cos(yaw) * vx + std::sin(yaw) * vy, -std::sin(yaw) * vx + std::cos(yaw) * vy, s.imu_gyro[2]};
        loco.setMeasuredBodyVelocity(v, true);
      }
      sim.cmd = loco.update(s, dt);
      sim.applyPD();
      mj_step(sim.m, sim.d);
      t += dt;
      { const double y = sim.yaw(); yaw_unwrapped += wrapAngle(y - yaw_prev); yaw_prev = y; }
      const double tilt = std::max(std::fabs(s.imu_rpy[0]), std::fabs(s.imu_rpy[1]));
      if (loco.mode() == LocoMode::WALK || loco.mode() == LocoMode::STAND) {
        max_tilt = std::max(max_tilt, tilt);
        min_z = std::min(min_z, sim.d->qpos[2]);
        if (sim.d->qpos[2] < 0.15 || tilt > 0.8) fell = true;
      }
      if (log && (int)std::round(t / dt) % 10 == 0) {
        const double yaw = sim.yaw();
        const double vx = sim.d->qvel[0], vy = sim.d->qvel[1];
        auto tel = loco.telemetry();
        std::fprintf(log, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f\n", t, sim.d->qpos[0],
                     sim.d->qpos[1], sim.d->qpos[2], yaw, s.imu_rpy[0], s.imu_rpy[1],
                     std::cos(yaw) * vx + std::sin(yaw) * vy, -std::sin(yaw) * vx + std::cos(yaw) * vy,
                     sim.d->qvel[5], (int)loco.mode(), tel.cmd_filtered.vx, tel.cmd_filtered.wz);
      }
      if (check_ready && loco.mode() == LocoMode::STAND && loco.standUpComplete()) break;
    }
  };

  for (auto& c : cmds) {
    std::stringstream ss(c);
    std::string op;
    ss >> op;
    const double x0 = sim.d->qpos[0], y0 = sim.d->qpos[1], yaw0 = sim.yaw(), t0 = t, yawu0 = yaw_unwrapped;
    if (op == "standup") {
      run_for(0.5);
      loco.requestStandUp();
      run_for(8.0, true);
      est.resetOdometry({sim.d->qpos[0], sim.d->qpos[1], sim.yaw()});
      std::printf("[%.2f] standup -> mode=%s complete=%d z=%.3f\n", t, toString(loco.mode()), loco.standUpComplete(),
                  sim.d->qpos[2]);
    } else if (op == "wait") {
      double T; ss >> T; run_for(T);
      std::printf("[%.2f] wait %.1f -> mode=%s z=%.3f pos=(%.3f,%.3f) yaw=%.3f\n", t, T, toString(loco.mode()),
                  sim.d->qpos[2], sim.d->qpos[0], sim.d->qpos[1], sim.yaw());
    } else if (op == "walk") {
      double vx, vy, wz, T; ss >> vx >> vy >> wz >> T;
      loco.setVelocityCommand({vx, vy, wz});
      loco.requestWalk(true);
      run_for(T);
      const double dx = sim.d->qpos[0] - x0, dy = sim.d->qpos[1] - y0;
      const double fwd = std::cos(yaw0) * dx + std::sin(yaw0) * dy;
      const double lat = -std::sin(yaw0) * dx + std::cos(yaw0) * dy;
      {
        auto e = est.legOdometryEstimate();
        const double yaw = sim.yaw(), vxw = sim.d->qvel[0], vyw = sim.d->qvel[1];
        std::printf("   odo pose=(%.3f,%.3f,%.3f) gt=(%.3f,%.3f,%.3f) | odo v=(%.3f,%.3f) gt v=(%.3f,%.3f) h=%.3f\n", e.pose.x,
                    e.pose.y, e.pose.yaw, sim.d->qpos[0], sim.d->qpos[1], yaw, e.vel_body.vx, e.vel_body.vy,
                    std::cos(yaw) * vxw + std::sin(yaw) * vyw, -std::sin(yaw) * vxw + std::cos(yaw) * vyw, e.height);
      }
      std::printf("[%.2f] walk(%.2f,%.2f,%.2f) %.1fs -> fwd=%.3f lat=%.3f dyaw=%.3f avg_v=%.3f avg_wz=%.3f z=%.3f mode=%s\n",
                  t, vx, vy, wz, T, fwd, lat, yaw_unwrapped - yawu0, fwd / (t - t0),
                  (yaw_unwrapped - yawu0) / (t - t0), sim.d->qpos[2], toString(loco.mode()));
    } else if (op == "stop") {
      loco.setVelocityCommand({0, 0, 0});
      loco.requestWalk(false);
      run_for(1.5);
      std::printf("[%.2f] stop -> mode=%s z=%.3f\n", t, toString(loco.mode()), sim.d->qpos[2]);
    } else if (op == "sit") {
      loco.requestSitDown();
      run_for(2.5);
      std::printf("[%.2f] sit -> mode=%s z=%.3f\n", t, toString(loco.mode()), sim.d->qpos[2]);
    } else if (op == "push") {
      double fx, fy, T; ss >> fx >> fy >> T;
      const int b = mj_name2id(sim.m, mjOBJ_BODY, "base_link");
      sim.d->xfrc_applied[6 * b] = fx; sim.d->xfrc_applied[6 * b + 1] = fy;
      run_for(T);
      sim.d->xfrc_applied[6 * b] = 0; sim.d->xfrc_applied[6 * b + 1] = 0;
      std::printf("[%.2f] push(%.0f,%.0f) %.2fs\n", t, fx, fy, T);
    }
  }
  std::printf("SUMMARY fell=%d max_tilt=%.3f min_z=%.3f final=(%.3f,%.3f,%.3f) yaw=%.3f\n", fell, max_tilt, min_z,
              sim.d->qpos[0], sim.d->qpos[1], sim.d->qpos[2], sim.yaw());
  if (log) std::fclose(log);
  return fell ? 2 : 0;
}
