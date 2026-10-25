# go2_autonomy — autonomous A→B navigation for the Unitree Go2 (C++, Unitree SDK2, unitree_mujoco)

This project makes the simulated Go2 in `unitree_mujoco` walk on its own from A to B,
detect obstacles and go around them. Everything runs through real DDS command/feedback:
`rt/lowcmd` out, `rt/lowstate` / `rt/sportmodestate` / a simulated range scan in.
There is no teleporting and no scripted pose changes. All motion is the result of 12 joint PD
commands produced by our own gait.

```cpp
go2::Go2NavigationSystem robot;   // sim defaults: DDS domain 1, interface "lo"
robot.start();
robot.standUp();
go2::NavResult r = robot.goTo(5.0, 0.0);   // blocking, obstacle-aware
// r.success / r.position_error are judged on the MEASURED pose after the robot has stopped
```

## 1. Where things are (in this repository)

| Path | What |
|---|---|
| `go2_autonomy/` | This project: library, apps and tools |
| `unitree_mujoco/simulate/nav_ext/` | `unitree_mujoco_nav`: the **stock simulator** plus a simulated range scanner published over DDS. Builds into `simulate/build_nav/` |
| `unitree_mujoco/unitree_robots/go2/scene_nav.xml` | Flat arena with 3 obstacles. It includes the **unmodified** `go2.xml` |
| `examples/stand_go2_3step/` | The original 3-step stand controller that the stand-up sequence is based on |

Apart from those two additions, `unitree_mujoco/` and `unitree_sdk2/` are unmodified upstream copies (see `THIRD_PARTY.md`).
`unitree_mujoco_nav` compiles the **unmodified** `simulate/src/main.cc` and only swaps the Go2 bridge
class for a subclass that also starts the scanner (see `nav_ext/main_nav.cc`). All LowCmd / LowState /
SportModeState behaviour is identical to `unitree_mujoco`.

## 2. Build

From the repository root, run `bash scripts/install_deps.sh` once, then `bash scripts/build.sh`.
The illustrated walk-through is in [`docs/GETTING_STARTED.md`](../docs/GETTING_STARTED.md).
Optional offline tuning tools that link MuJoCo directly:
`GO2_CMAKE_ARGS="-DGO2_BUILD_TOOLS=ON -DMUJOCO_DIR=$PWD/third_party/mujoco-3.3.6" bash scripts/build.sh`

## 3. Run

```bash
bash scripts/run_sim.sh                     # terminal 1: simulator with scanner + obstacle arena
bash scripts/run_nav_demo.sh                # terminal 2: A (start) -> B (5, 0) around the obstacles
bash scripts/run_acceptance_tests.sh        # terminal 2: the 8 acceptance tests (about 80 s)
go2_autonomy/build/go2_scan_probe           # check lowstate / sportmodestate / scan are alive
```
`go2_nav_app` refuses DDS domain 0 (the real robot) unless explicitly overridden. See `docs/HARDWARE_NOTES.md`.
Restart the simulator (or press Backspace in its window to reset) before each run. The robot has to start lying at the origin.
Plots and a replay video from a test log:
`python3 go2_autonomy/scripts/plot_run.py test_logs/<run>` and
`MUJOCO_GL=egl python3 go2_autonomy/scripts/render_replay.py test_logs/<run> unitree_mujoco/unitree_robots/go2/scene_nav.xml out.mp4 --tests 5_,6_ --track`
(needs `pip install mujoco==3.3.6 imageio matplotlib`).

## 4. What the existing environment actually provides (inspection results)

| Signal | Source | Content | Notes |
|---|---|---|---|
| `rt/lowstate` (`unitree_go::LowState_`) | `unitree_sdk2_bridge.h` | 12× q, dq, tau_est; IMU quaternion, rpy, gyro, accelerometer; tick | `foot_force` is **always 0** in MuJoCo: no contact sensing |
| `rt/sportmodestate` (`SportModeState_`) | same bridge | `position`, `velocity` of the IMU site = **simulator ground truth** | On the real robot this topic is **not published** once the sport service is released for low-level control (unitree_mujoco readme) |
| `rt/lowcmd` (`LowCmd_`) | bridge applies `tau + kp(q*-q) + kd(dq*-dq)` at 1 kHz | per-motor q, dq, kp, kd, tau | MuJoCo motor order: FR, FL, RR, RL × (hip, thigh, calf), the same as on hardware |
| Obstacle sensing | **none** in stock unitree_mujoco | — | added: `rt/sim/range_cloud` (see §5.5) |

Physics: MuJoCo 3.3.6, dt = 2 ms, foot friction pairs with the floor (μ = 1), torque limits 23.7 / 23.7 / 45.4 Nm.

## 5. Architecture

```
 apps/  go2_nav_app · go2_acceptance_tests · go2_scan_probe          (business logic / tests)
   │
 app/   Go2NavigationSystem  ── goTo(x, y[, yaw]) ──┐
   │                                                 │
 navigation/ Navigator (20 Hz)                      │  NavRobotInterface  (estimate, scan,
   ├─ perception/ OccupancyGrid  (log-odds, height classification, inflation)   setVelocity, setWalking)
   ├─ planning/   PathPlanner    (A* on cost map + line-of-sight smoothing, replanning)
   └─ pure-pursuit follower + reactive stop zone on the raw scan
   │  body twist (vx, vy, wz)
 app/   Go2Robot (500 Hz control thread, watchdog)
   ├─ estimation/ StateEstimator  (sportmodestate pose | leg-inertial odometry)
   ├─ locomotion/ LocomotionController (stand-up/stand/trot/sit, IK, PD, feed-forward)
   │     └─ robot/ Go2Kinematics (FK / IK / Jacobian, verified against go2.xml)
   └─ comm/ Go2DdsInterface  (Unitree SDK2: rt/lowcmd ⇄ rt/lowstate, sportmodestate, scan)
                     │ DDS (domain 1, lo)
              unitree_mujoco_nav  =  stock unitree_mujoco + RangeScanPublisher
```

`go2_core` (kinematics, locomotion, estimation, perception, planning) has **no DDS dependency**, so
it can be unit-tested offline (`tools/mujoco_harness`) and reused unchanged on hardware.

### 5.1 Locomotion (`locomotion_controller.cpp`)
* **Stand-up / sit-down** follows `examples/stand_go2_3step/stand_go2_3step_ik_pd_stable.cpp`: crouch and stand poses come from
  IK, cosine interpolation starts from the *measured* joints, and the sequence finishes only when the
  encoders have stopped moving (same 100 ms / 0.01 rad / 2 windows test). The crouch height of 0.1457 m comes from that controller.
* **Trot** (FR+RL / FL+RR), period 0.36 s, duty 0.5, step height 8 cm, foot 0.30 m below the hip.
  Stance feet move against the commanded twist (vx, vy, wz). Swing uses a cycloid profile towards a
  Raibert touchdown point.
* **Feedback** (all from `rt/lowstate`): a yaw-rate PI on the IMU gyro (removes the trot's yaw drift and
  makes turning accurate), velocity feedback in foot placement plus a small integral from *leg odometry*,
  attitude levelling from IMU roll/pitch, and gravity feed-forward `τ = Jᵀf` on stance legs.
  The joint PD is done by the motor driver/bridge (stance kp 60 / kd 2, swing 45 / 1.5).
* **Transitions**: stand→walk restarts the gait phase from double support. walk→stand ramps the command
  to zero and switches only at a touchdown boundary, then blends the feet to nominal over 0.4 s.
* Measured in MuJoCo: 0.4 m/s command gives 0.43 m/s with < 2 cm lateral drift over 1.7 m. ±0.8 rad/s gives
  ±0.78 rad/s. 0.7 m/s is stable. Max tilt while walking is about 0.02–0.07 rad.

### 5.2 State estimation (`state_estimator.cpp`)
* `SPORT_STATE` (default in simulation): position from `rt/sportmodestate`, yaw from the IMU quaternion.
* `LEG_ODOMETRY` (hardware-portable, `--pose odom`): stance-leg velocity `v = −(J q̇ + ω×r)`, with
  rolling-foot correction, levelled with IMU attitude and integrated with IMU yaw. Drift in simulation
  is about 3 % of distance when walking straight, and about 8 % over a full mission with turns (see §6.2).

### 5.3 Perception and planning
* Body-frame points → levelled with IMU roll/pitch → height above ground from leg kinematics.
  0.07–0.85 m counts as an obstacle, < 0.04 m as ground (free). A cell is cleared only along the part of a
  ray that passes **below 0.12 m**, so a high beam over a low box cannot erase it.
* Log-odds grid, 5 cm, 14 × 12 m. Brushfire distance transform: lethal within 0.34 m, soft cost to 0.70 m.
* A* (8-connected) with a clearance-weighted cost and line-of-sight smoothing. Replans every 1 s, at once if
  the current path crosses newly discovered obstacles, and fails after 10 s without any valid path.
* Follower: pure pursuit (0.55 m lookahead). The robot turns in place for heading errors > 0.75 rad and
  slows near obstacles and the goal. A reactive stop zone (0.30 m ahead of the body) on the raw scan vetoes forward motion.
* Success = within 0.15 m (and yaw if requested) **after the gait has stopped**, judged on the measured
  pose. Failure reasons: no path, no progress (25 s), timeout, locomotion fault.

### 5.4 Safety behaviour (built in)
* **Communication watchdog:** if `rt/lowstate` stops or its tick freezes for more than 0.25 s, the robot goes to damping
  (kp = 0, kd = 3). The fault latches until restart, and `standUp()` refuses while it is latched.
* **Damping or sit** also clears any pending walk or velocity request, so the robot can never resume walking on its own.
* **Navigation never drives blind.** It stops when the scan is older than 0.5 s or the pose is stale, and the goal
  fails after 2 s. A reactive stop zone on the raw scan vetoes forward motion.
* **`Go2Robot::stop()`**, the destructors and Ctrl+C in `go2_nav_app` all do the same thing: cancel the goal, stop the gait at a
  touchdown, command damping for 0.3 s, then join the control thread. The last command left in the drivers
  is therefore damping, never a mid-stride pose.
* `go2_nav_app` refuses DDS domain 0 (the real robot) unless explicitly overridden. `go2_acceptance_tests` always refuses it.

### 5.5 Simulated range scanner (`simulate/nav_ext/range_scan_publisher.h`)
Three layers (0°, −10°, −20°) × 180 azimuths (2°), mounted 0.10 m above `base_link`, 6 m range,
σ = 1 cm noise, 10 Hz. Published as `sensor_msgs::PointCloud2` (x, y, z float32, base_link frame) on
`rt/sim/range_cloud`. Rays are cast with `mj_ray` on a private `mjData` rebuilt from a snapshot of `qpos`,
so the physics thread is never blocked. Only relative geometry is published, never the world pose.
The tilted layers detect the 0.25 m low box, which the horizontal layer passes over.

## 6. Verification

### 6.1 Acceptance tests over DDS (`go2_acceptance_tests`, scene_nav.xml, pose = sportmodestate)
Six independent full runs (the last two after the code-review fixes), **8/8 PASS each time**. Numbers from the
final run (`docs/validation/summary.txt`, `console.log`, `trajectory.csv`, `run_final.png`, `go2_avoidance_replay.mp4`):

| # | Test | Measured result |
|---|---|---|
| 1 | Stands stably | encoder-settled stand-up; max roll/pitch 0.009 rad over a 3 s hold; drift 2 mm |
| 2 | Straight line (forward and backward) | 0.40 m/s cmd → 1.73 m in 4 s, lateral −13 mm, heading 0.000 rad; −0.30 m/s → −1.02 m |
| 3 | Turns | +0.8 rad/s × 4 s → +3.13 rad; −0.8 rad/s × 4 s → −3.14 rad; 2 cm drift |
| 4 | Free-space goal (0, 2.5) | SUCCESS in 8.5 s, measured error 0.075 m |
| 5 | Detects blocking obstacle | 72 wall cells mapped from A; straight A→B blocked in the cost map; 0 false cells |
| 6 | Navigates around | 1.83 m lateral detour; min footprint clearance 0.28 m (independent oracle); 4 replans on newly seen obstacles |
| 7 | Reaches B (5, 0) | SUCCESS in 20.8 s, 6.6 m travelled, measured error 0.073 m |
| 8 | Reports success/failure | goal inside the pillar → FAILURE "no collision-free path" after 10 s, no collision; test 7 SUCCESS agrees with the measured pose |

The collision oracle in the test app knows the obstacle geometry of `scene_nav.xml`. The navigation stack
never does. A 4-goal mission with `go2_nav_app` (5,0) → (0,0,π) → (4.6,2.3) → (1,−2) also succeeded
(measured errors 0.07–0.08 m, `docs/validation/nav_app_console.log`).
Control loop in the final run: 39 148 cycles at 500 Hz, every one followed by an `rt/lowcmd`; real-time factor 1.00.

**Safety behaviour** (`docs/validation/safety_tests.log`):
* Stock `unitree_mujoco` (no scanner): `goTo` refuses to move and fails after 2 s with "no range-sensor data received".
* Simulator frozen for 1 s (SIGSTOP) while walking: the lowstate watchdog trips at 0.25 s, the robot goes to damping, and the goal fails with "locomotion not ready".
* Ctrl+C while walking: the goal is cancelled, the gait stops at a touchdown, the robot is damped, and the process exits cleanly.
* `test_planner` (offline unit test): detour around a wall; escape from inside an inflation halo without planning through the wall; a low box is not erased by rays passing above it.

**Code review.** An independent review pass found 9 issues (mostly in fault paths), and all of them are fixed:
* the robot could resume walking after damping
* navigation did not detect stale scans or pose
* the control thread was not joined at stop
* navigator shutdown left the robot walking
* the A* escape could cut through walls
* foot targets snapped when restarting the gait
* the logger ran under the navigator lock
* point-cloud layout was not bounds-checked
* a paused simulator was not detected

**SDK finding you should know about.** Calling `ChannelSubscriber::CloseChannel()` (or destroying a
subscriber) while its 1 kHz `rt/lowstate` callbacks are running aborts the process with an assertion inside the
SDK's CycloneDDS-C++ library (`EntityDelegate::prevent_callbacks`). In a 30-close stress test this happened in 3 of 5 processes.
`Go2DdsInterface` therefore never destroys its readers. This also matters for any other SDK program that closes channels.

### 6.2 With leg odometry only (`--pose odom`, no ground truth in the loop)
5/8. Avoidance still works (no collision, min clearance 0.28 m) because the map is built in the same
drifting frame. But the world-frame goal error grows: 0.23 m after about 8 m of travel, 0.56 m at B. Test 8 then
"succeeds" at the pillar coordinates, because the odometry frame is offset from the world. Conclusion: on a real robot,
navigation needs an external localisation source (LiDAR odometry/SLAM). See `docs/HARDWARE_NOTES.md`.

### 6.3 What was NOT verified
* **Workstations with a GPU.** All results above come from an Ubuntu 24.04 cloud container (2 CPUs, software
  rendering) using the exact versions in this repository. The code was also compiled with gcc 11 (Ubuntu 22.04's
  compiler) and built on the Ubuntu 22.04 development PC, but the acceptance tests are recorded only from the
  container. Run `bash scripts/run_acceptance_tests.sh` once on each new machine.
* **Real hardware.** Nothing here has run on a physical Go2.
* **Timing.** On the 2-core container the 500 Hz loop had about 1–2 % late cycles (max 58 ms stall) because of
  software rendering. Expect much better on a normal workstation. A stall above 250 ms on `rt/lowstate` triggers damping.
* Terrain and stairs (`scene_terrain.xml`), dynamic obstacles, pushes, slopes, and speeds above 0.7 m/s were not tested.

## 7. Tuning

`GaitParams` (locomotion), `NavConfig`, `GridConfig`, `PlannerConfig` are plain structs passed via
`SystemConfig`. For fast offline gait tuning without DDS:
```bash
./build/mujoco_harness ../unitree_mujoco/unitree_robots/go2/scene_nav.xml \
  "standup;walk 0.5 0 0 6;walk 0 0 0.8 4;stop"        # env: GAIT="T=0.36,sh=0.08,kps=60" VFB=2
./build/test_kinematics ../unitree_mujoco/unitree_robots/go2/go2.xml   # FK/IK vs MuJoCo
./build/test_planner                                                            # grid + A* unit test (always built)
```
