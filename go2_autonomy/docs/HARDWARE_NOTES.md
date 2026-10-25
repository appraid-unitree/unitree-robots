# Moving go2_autonomy to a physical Go2: what carries over, what does not

Nothing in this repository has run on a physical robot. This page lists what has to change and what has to be
validated before it does. Read it before pointing any binary at `--domain 0` or a robot network interface.

## 1. Component portability

| Layer | Module | On a real Go2 | Status |
|---|---|---|---|
| Business app | `apps/`, `Go2NavigationSystem` | reusable | API unchanged |
| Navigation | `Navigator`, `OccupancyGrid`, `PathPlanner` | reusable | depends only on `NavRobotInterface` |
| Robot interface | `NavRobotInterface` | reusable | add an adapter for Unitree sport mode (see §3) |
| Comm | `Go2DdsInterface` | reusable (domain 0, robot NIC) | LowCmd/LowState/CRC identical to the SDK examples |
| Kinematics | `Go2Kinematics` | reusable | geometry identical to the official URDF |
| Estimation | leg odometry | reusable, but drifts (3–8 %) | **needs an external localisation source** |
| Estimation | `SPORT_STATE` source | **not available** | `rt/sportmodestate` stops when the sport service is released |
| Perception input | `rt/sim/range_cloud` | **replace** | use the L1 LiDAR (`rt/utlidar/cloud`, PointCloud2) through a frame adapter |
| Locomotion | `LocomotionController` | **unvalidated** | tuned only in MuJoCo, see §2 |

## 2. Running our own low-level gait on hardware (highest risk)
1. Release the built-in controller first, the same way the SDK example `example/go2/go2_stand_example.cpp` does
   (`MotionSwitcherClient::CheckMode/ReleaseMode`). Otherwise two controllers write motor commands at once.
2. Use domain 0 and the robot Ethernet interface. `go2_nav_app` refuses domain 0 unless you pass
   `--real-robot-i-have-read-hardware-notes`. `go2_acceptance_tests` is simulation-only and always refuses it.
3. Things the simulator does not model: motor bandwidth and latency, gearbox friction, DDS jitter on a
   real network, foot slip on real floors, battery sag. Re-tune `kp/kd`, period, step height and the
   feed-forward torque limits with the robot **suspended**, then on a harness.
4. Use `foot_force` from `rt/lowstate` (always 0 in MuJoCo) for real contact detection in odometry and gait.
5. Safety: keep the lowstate watchdog (→ damping) and add a hardware e-stop path
   (wireless controller key → `requestDamping()`). Start with `v_max ≤ 0.2 m/s`.

## 3. Recommended commercial path (lowest risk)
Implement `NavRobotInterface` on top of Unitree's **built-in** locomotion:
`setVelocity()` → `unitree::robot::go2::SportClient::Move(vx, vy, wz)`, `setWalking(false)` → `StopMove()`,
`estimate()` → `rt/sportmodestate` (available in this mode) or LiDAR odometry. Then the navigation
layer can be used on the robot without our gait. The custom low-level gait stays an R&D track.

## 4. Localisation and sensing on the robot (to verify on the unit)
* Check which of `rt/utlidar/cloud`, `rt/utlidar/robot_odom` and `rt/utlidar/robot_pose` your firmware publishes,
  and in which frame and at what rate. Check whether they keep publishing with the sport service released.
* Write a `PointCloud` adapter that converts the LiDAR cloud into the body frame
  (head mount, tilt, deskew). The grid expects body-frame points plus roll/pitch/height.
* Without an external pose, expect goal errors of about 0.2–0.6 m over 5–10 m of travel (measured in simulation with `--pose odom`).
