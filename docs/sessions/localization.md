# Session: localization

Read `docs/CONTRACTS.md` first. You own `/team/odom_map`, TF `course → map`, and `/team/loc/status`.

## Goal

A pose in the `course` frame that stays accurate for the whole run (window, landing ring, return to
within 2 ft of launch all depend on it), never stalls, and says honestly when it is degraded.

## Starting point

cuVSLAM is already running on the drone and feeding PX4. Treat it as the motion source (good short-term,
drifts long-term) and add **drift correction against the known map**. Don't replace what PX4 uses: your
correction lives in `course → map` only.

## Proven open-source options (experiment, keep what measures best)

Drift correction, pick one first, compare others later:

- **2D walls:** depth → laser scan ([depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan)
  or `pointcloud_to_laserscan`) matched to the occupancy grid with [nav2_amcl](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
  or scan-to-map ICP. Cheap, robust in corridors, walls are 68 in tall so a horizontal slice always hits them.
- **3D registration:** depth point cloud → [small_gicp](https://github.com/koide3/small_gicp) against the course
  point cloud `planner/maps/course_v3.bin`. Pattern: [small_gicp_relocalization](https://github.com/Alex10086/small_gicp_relocalization)
  publishes a map-to-odom correction from exactly this.
- **Visual landmarks:** detect course features (gates, window, tunnel mouth, pad) with
  [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) trained on renders of the course model, then
  bearing/PnP measurements into the filter. This is what top racing teams do
  ([drift-corrected VIO, 2025](https://arxiv.org/abs/2512.20475)). Train on renders only (rule 6.4).

Fusion: [robot_localization](https://github.com/cra-ros-pkg/robot_localization) EKF to start, or
[GTSAM](https://github.com/borglab/gtsam) iSAM2 if you go factor-graph.

Depth is **not** published on the drone today. Options: GPU stereo on the Jetson from `/oak/*/image_mono`
(e.g. VPI or [isaac_ros_ess](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)), OpenCV SGBM at
low resolution, or ask the organizers about on-camera depth. Budget GPU: cuVSLAM already uses it.

Alternative VIO to benchmark only if cuVSLAM struggles (dim tunnel, speed):
[OpenVINS](https://github.com/rpng/open_vins) stereo. Also test cuVSLAM camera exposure/gain in low light.

## Things to prove in sim

- Drift: error vs Gazebo ground truth (`/sim/ground_truth/odom`) along the full course, at the pad, back at launch.
- Robustness: dim tunnel lighting; fast motion; a forced VIO dropout (does the correction recover?).
- Latency: `/team/odom_map` must not lag VIO by more than a few ms; it is in the control loop.
- Compute: CPU and GPU load must fit on the Orin Nano next to cuVSLAM.
- Honest status: WARN/ERROR must trigger before the error gets large.

## Milestones

1. Republisher: `/team/odom_map` + TF from cuVSLAM and the static launch pose (unblocks path following on day one).
2. Evaluation script: ATE/drift vs ground truth from a rosbag, plotted per checkpoint.
3. First drift correction (2D walls is the fastest). Show it beats raw VIO on the full course.
4. Depth source that runs on the Jetson within budget.
5. Second method (3D or landmarks); keep whichever measures best, or fuse both.
6. Dim-tunnel and dropout scenarios pass.

Keep a results table in `docs/results/localization.md`.

## Default prompt for this session

> You are the localization workstream for our Stanford DroneHacks 2026 team.
> Read `docs/CONTRACTS.md` and `docs/sessions/localization.md` in this repo, then `docs/SIM.md`.
> Build ROS 2 Humble packages under `src/` that take cuVSLAM odometry and correct its drift against the
> known course map, publishing `/team/odom_map`, TF `course → map` and `/team/loc/status` exactly as the
> contracts say. Ship milestone 1 (the republisher) first so path following is unblocked. Use the proven
> open-source libraries in the doc rather than writing estimators from scratch, and measure every method
> against Gazebo ground truth on the GCP sim (your slot is 1), recording results in `docs/results/localization.md`.
> Nothing may use data recorded on the real course in a later run. Do not modify `~/skybeta`. Work on
> branch `localization` off `sim-and-plans`, commit small, and open a PR when a milestone works in sim.

## Drone-ready status (2026-11-07 evening)

**Recommended drone config: M1.** The republisher, with the correction and the matcher off. Jump
absorption and the VIO sanity checks are always on.

```bash
ros2 launch team_loc team_loc.launch.py \
    course_config:=/root/team_ws/src/<repo dir>/planner/config/<the course config the plan uses> \
    base_height_on_pad_m:=<tape: floor to the Pixhawk IMU, drone on the pad>
# defaults: enable_correction:=false with_matcher:=false
```

Build, checks and CPU are in `docs/DRONE_RUN.md`. The hand-carry check needs `ignore_land_detector:=true`.

What M1 does:
- Anchors on the pad from StartPos/StartHeadingDeg plus PX4 roll and pitch, until real takeoff.
- Publishes `/team/odom_map` (frame `course`) on every VIO message, plus TF `course → map` and
  `/team/loc/status`.
- **Absorbs VIO jumps:** the course pose stays continuous and keeps the last good velocity; WARN for 3 s.
- ERROR on lost, stale or frozen VIO. Costs about 0.5 % of one core.

Gazebo slot 1 numbers, ATE = horizontal RMS vs ground truth:

| run | flown on | correction | ATE corrected / raw cuVSLAM | checkpoints corrected / raw (m) | notes |
|---|---|---|---|---|---|
| gt08 | ground truth | on (passive) | 0.058 / 0.084 m | 0.04 0.04 0.03 0.11 0.10 / 0.05 0.05 0.06 0.13 0.10 | after the net-model fix; RTF ≈ 0.3 |
| loc01 | /team/odom_map | on | 0.059 / 10.9 m | 0.04 0.03 0.03 0.08 0.21 / 0.02 0.02 0.02 0.00 21.4 | cuVSLAM blew up near the goal (about 3 m per frame for 0.3 s); our pose stayed continuous and ended 3 cm off. Absorption now keeps the dropped motion |
| loc02 | /team/odom_map | on | 0.059 / **0.027** m | 0.03 0.01 0.05 0.08 0.02 0.04 0.05 / 0.03 0.04 0.01 0.02 0.05 0.01 0.03 | candidate 2 (Gate B, sphere 0.35, time_scale 1.0); RTF 0.34 in flight, 1.0 at end. cuVSLAM was very good, and the correction added error. True min clearance 0.278 m |
| gt10 | ground truth | on (passive) | not finished at wind-down | | |

In every slot-1 run with the correction on, status was never OK while error was above 0.3 m, and
ERROR never fired in flight.

**Why the correction stays off:** it helps when cuVSLAM drifts or jumps (gt08, loc01) and hurts when
cuVSLAM is good (loc02). Its fixes are noisier (median about 5–8 cm in flight) than good VIO (about
3 cm). Next steps:
- weight fixes less and only correct on a significant disagreement;
- find the in-flight matcher error (scan stacking, attitude at the image stamp);
- run 3 or more repeats at RTF 1.0.

**Spot (0.65, −1.12),** where path following's c2rep1 strict-touched, from loc02 (34 samples within 0.6 m):
- Localization error there was small: 2.6 cm corrected (max 4.4 cm), 3.4 cm raw (max 4.8 cm).
- The true clearance was **0.286 m** at (0.66, −1.19, 1.14), just inside the 0.29 m prop reach.
- So the near-touch there is a **planning/tracking margin**, not localization error.
