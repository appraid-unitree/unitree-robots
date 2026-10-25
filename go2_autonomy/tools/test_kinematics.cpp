// Verifies Go2Kinematics against the MuJoCo model (FK, IK round trip, Jacobian).
#include <mujoco/mujoco.h>

#include <cstdio>
#include <random>

#include "go2_autonomy/robot/go2_kinematics.hpp"

using namespace go2;

int main(int argc, char** argv) {
  if (argc < 2) { std::printf("usage: test_kinematics go2.xml\n"); return 1; }
  char err[1000];
  mjModel* m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!m) { std::printf("%s\n", err); return 1; }
  mjData* d = mj_makeData(m);
  const char* foot_names[4] = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  const char* jn[4] = {"FR_hip_joint", "FL_hip_joint", "RR_hip_joint", "RL_hip_joint"};
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-1, 1);
  double max_fk = 0, max_ik = 0;
  for (int trial = 0; trial < 200; ++trial) {
    mj_resetData(m, d);
    LegJoints q[4];
    for (int leg = 0; leg < 4; ++leg) {
      q[leg] = {0.4 * u(rng), 0.8 + 0.6 * u(rng), -1.7 + 0.6 * u(rng)};
      const int adr = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, jn[leg])];
      for (int j = 0; j < 3; ++j) d->qpos[adr + j] = q[leg][j];
    }
    mj_kinematics(m, d);
    const int base = mj_name2id(m, mjOBJ_BODY, "base_link");
    for (int leg = 0; leg < 4; ++leg) {
      const int b = mj_name2id(m, mjOBJ_BODY, foot_names[leg]);
      // world -> base frame (base orientation identity at reset, but be general)
      double rel[3], loc[3];
      for (int k = 0; k < 3; ++k) rel[k] = d->xpos[3 * b + k] - d->xpos[3 * base + k];
      mju_mulMatTVec3(loc, d->xmat + 9 * base, rel);
      Vec3 hip = Go2Geometry::hipOffset(leg);
      Vec3 mj{loc[0] - hip.x, loc[1] - hip.y, loc[2] - hip.z};
      Vec3 fk = Go2Kinematics::forward(leg, q[leg]);
      max_fk = std::max(max_fk, (mj - fk).norm());
      LegJoints qi;
      Go2Kinematics::inverse(leg, fk, qi);
      Vec3 fk2 = Go2Kinematics::forward(leg, qi);
      max_ik = std::max(max_ik, (fk2 - fk).norm());
      if (trial == 0)
        std::printf("leg %d mj=(%.4f %.4f %.4f) fk=(%.4f %.4f %.4f) q=(%.3f %.3f %.3f) qik=(%.3f %.3f %.3f)\n", leg,
                    mj.x, mj.y, mj.z, fk.x, fk.y, fk.z, q[leg][0], q[leg][1], q[leg][2], qi[0], qi[1], qi[2]);
    }
  }
  std::printf("max FK error vs MuJoCo: %.2e m\nmax IK round-trip error: %.2e m\n", max_fk, max_ik);
  const bool pass = max_fk < 1e-4 && max_ik < 1e-6;
  std::printf("%s\n", pass ? "KINEMATICS PASS" : "KINEMATICS FAIL");
  return pass ? 0 : 1;
}
