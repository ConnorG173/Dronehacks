# Session: localization

Read `docs/CONTRACTS.md` first. You own `/team/odom_map`, TF `map → odom`, and `/team/loc/status`.

## Goal

A pose in the course `map` frame that stays accurate for the whole run (window, landing ring, return to
within 2 ft of launch all depend on it), never stalls, and says honestly when it is degraded.

## Starting point

cuVSLAM is already running on the drone and feeding PX4. Treat it as the motion source (good short-term,
drifts long-term) and add **drift correction against the known map**. Don't replace what PX4 uses: your
correction lives in `map → odom` only.

## Proven open-source options (experiment, keep what measures best)

Drift correction, pick one first, compare others later:

- **2D walls:** depth → laser scan ([depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan)
  or `pointcloud_to_laserscan`) matched to the occupancy grid with [nav2_amcl](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
  or scan-to-map ICP. Cheap, robust in corridors, walls are 68 in tall so a horizontal slice always hits them.
- **3D registration:** depth point cloud → [small_gicp](https://github.com/koide3/small_gicp) against the course
  point cloud `planner/maps/course_v3.bin`. Pattern: [small_gicp_relocalization](https://github.com/Alex10086/small_gicp_relocalization)
  publishes `map → odom` from exactly this.
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
> known course map, publishing `/team/odom_map`, TF `map → odom` and `/team/loc/status` exactly as the
> contracts say. Ship milestone 1 (the republisher) first so path following is unblocked. Use the proven
> open-source libraries in the doc rather than writing estimators from scratch, and measure every method
> against Gazebo ground truth on the GCP sim (your slot is 1), recording results in `docs/results/localization.md`.
> Nothing may use data recorded on the real course in a later run. Do not modify `~/skybeta`. Work on
> branch `localization` off `sim-and-plans`, commit small, and open a PR when a milestone works in sim.
