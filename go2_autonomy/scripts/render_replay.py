#!/usr/bin/env python3
"""Offline replay renderer for an acceptance-test log.

Re-poses the Go2 MuJoCo model from the MEASURED feedback recorded during the
run (rt/sportmodestate position + rt/lowstate IMU quaternion + 12 joint
encoders) and renders it with MuJoCo's offscreen renderer. No physics is
simulated here -- it is a faithful visual replay of what the simulator reported.

usage: MUJOCO_GL=osmesa render_replay.py <log_dir> <scene.xml> <out.(mp4|gif)>
       [--tests 5_,6_] [--speed 2.0] [--fps 15] [--size 960x540]
"""
import argparse, csv, math, os
import numpy as np
import mujoco
import imageio.v2 as imageio

ap = argparse.ArgumentParser()
ap.add_argument("log_dir"); ap.add_argument("scene"); ap.add_argument("out")
ap.add_argument("--tests", default="")
ap.add_argument("--speed", type=float, default=2.0)
ap.add_argument("--fps", type=int, default=15)
ap.add_argument("--size", default="960x540")
ap.add_argument("--track", action="store_true", help="camera follows the robot")
a = ap.parse_args()
W, H = map(int, a.size.split("x"))

rows = list(csv.DictReader(open(os.path.join(a.log_dir, "trajectory.csv"))))
rows = [r for r in rows if "q11" in r and r["q11"] not in (None, "")]
if a.tests:
    keys = a.tests.split(",")
    rows = [r for r in rows if any(r["test"].startswith(k) for k in keys)]
assert rows, "no rows with joint data"

m = mujoco.MjModel.from_xml_path(a.scene)
m.vis.global_.offwidth = max(m.vis.global_.offwidth, W)
m.vis.global_.offheight = max(m.vis.global_.offheight, H)
d = mujoco.MjData(m)
r = mujoco.Renderer(m, H, W)
motor_joints = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]
adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in motor_joints]
imu_off = np.array([-0.02557, 0.0, 0.04232])

cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [2.3, -0.3, 0.0]
cam.distance = 7.3
cam.azimuth = 0
cam.elevation = -48

t0 = float(rows[0]["t"])
dt_frame = a.speed / a.fps
frames, trail = [], []
next_t = t0
for row in rows:
    t = float(row["t"])
    trail.append((float(row["gt_x"]), float(row["gt_y"])))
    if t < next_t:
        continue
    next_t += dt_frame
    q = np.array([float(row[k]) for k in ("qw", "qx", "qy", "qz")])
    q /= np.linalg.norm(q)
    R = np.zeros(9); mujoco.mju_quat2Mat(R, q); R = R.reshape(3, 3)
    site = np.array([float(row["gt_x"]), float(row["gt_y"]), float(row["gt_z"])])
    d.qpos[0:3] = site - R @ imu_off
    d.qpos[3:7] = q
    for i, ad in enumerate(adr):
        d.qpos[ad] = float(row[f"q{i}"])
    mujoco.mj_kinematics(m, d)
    if a.track:
        target = np.array([d.qpos[0], d.qpos[1], 0.15])
        if not frames:
            cam.lookat[:] = target; cam.distance = 3.6; cam.azimuth = 20; cam.elevation = -30
        cam.lookat[:] = 0.85 * np.array(cam.lookat) + 0.15 * target
    r.update_scene(d, cam)
    scn = r.scene
    for (x, y) in trail[::10]:  # breadcrumb trail of measured positions
        if scn.ngeom >= scn.maxgeom: break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]), np.array([x, y, 0.02]),
                            np.eye(3).flatten(), np.array([0.9, 0.1, 0.1, 0.9], dtype=np.float32))
        scn.ngeom += 1
    img = r.render().copy()
    frames.append(img)

kw = dict(fps=a.fps)
if a.out.endswith(".gif"):
    kw = dict(duration=1.0 / a.fps, loop=0)
    frames = [f[::2, ::2] for f in frames]  # half resolution for GIF size
imageio.mimsave(a.out, frames, **kw)
print(f"wrote {a.out}: {len(frames)} frames, {len(rows)} log rows, {a.speed}x real time")
