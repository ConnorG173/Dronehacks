# Localization on drone 12: what to build, launch and check (tonight)

Branch `localization`. Packages are `src/team_loc` (C++, node `loc_fuser`) and `src/team_loc_matcher`
(Python, node `wall_matcher`). Contract: `docs/CONTRACTS.md` (frame `course`, TF `course → map`).

## 1. Build

Both packages go in `~/team_ws/src` with the rest of the repo. That happens anyway when the repo is
cloned there; colcon finds them under `src/`.

Inside the stack container (`host/run.sh shell`, or the organizers' `team_build.sh`):

```bash
source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash
cd /root/team_ws && colcon build --packages-select team_loc team_loc_matcher --cmake-args -DBUILD_TESTING=OFF
```

- **Dependencies:** none to add. All are already in the drone's `skybeta:0.3.2` image (checked
  read-only on skybeta-12): rclcpp, tf2_ros, tf2_eigen, eigen3_cmake_module, nav_msgs,
  diagnostic_msgs, message_filters, tf2_ros_py, `px4_msgs` (in `/root/skybeta_ws/install`, so source
  it first), `isaac_ros_visual_slam_interfaces`, OpenCV 4.5.4, NumPy 1.26.4.
- **Build time:** `team_loc` (C++) takes about 1.5 min on the sim VM; expect 2–4 min on the Orin.
  `team_loc_matcher` is pure Python.
- **Never add to the workspace:** `px4_msgs`, or the stub `isaac_ros_visual_slam_interfaces` that
  `tools/loc_dev/build_test.sh` makes. That stub lives only in `~/.cache/team_loc_dev` on a laptop.

## 2. Launch (tonight: passive, M1)

```bash
ros2 launch team_loc team_loc.launch.py \
    course_config:=/root/team_ws/src/<repo dir>/planner/config/course_v3.yaml \
    base_height_on_pad_m:=<measured, see below>
```

The defaults are `enable_correction:=false with_matcher:=false`. **Recommendation for tonight: keep
both false.**

What runs: `loc_fuser` only. It anchors on the pad from `StartPos`/`StartHeadingDeg`, using PX4
attitude for roll and pitch. It then republishes every cuVSLAM message as `/team/odom_map` (frame
`course`, about 30 Hz), plus TF `course → map` and `/team/loc/status`. It never publishes to PX4 or
to setpoints.

Why no correction yet:
- It wins clearly offline (closed loop through the course map: end error 2–4 cm vs 0.26–1.4 m).
- In slot 1 on rendered stereo, most scans failed the matcher's score gate. The coordinator has since
  found that the sim course had only 6 tiled textures. The fix is in (`sim-and-plans` 3dedfc0) but not
  yet re-measured.
- I will message the coordinator if it passes the gate in slot 1. Turning it on later is two launch
  arguments: `enable_correction:=true with_matcher:=true`.

What M1 still does for you tonight:
- **The anchor** follows the drone on the pad until real takeoff (PX4 not-landed and 10 cm of
  motion), so carrying or creep before takeoff does no harm.
- **VIO jumps** are absorbed into the anchor, so the course pose stays continuous. Status goes WARN for 3 s.
- **Honest status:** ERROR on `vo_state` 2, VIO older than 0.2 s, not anchored, or frozen VIO
  (static pose while the rangefinder moves). WARN on VIO rate < 20 Hz, a static pose in flight, or a
  rangefinder vs VIO height disagreement > 0.35 m over 1 s.

**`base_height_on_pad_m`** is the height of `base_link` above the floor with the drone on the pad.
`base_link` is the **Pixhawk 6C's primary IMU** (the organizers' frames contract). Measure it with a
tape: floor to the middle of the Pixhawk case, on the pad. It only shifts `/team/odom_map` z. If it is
left at 0, z reads that much low, and a tracker that holds z from `/team/odom_map` flies that much
high.

## 3. CPU / GPU next to cuVSLAM

| Node | Cost | Source |
|---|---|---|
| `loc_fuser` | 0.16 ms p50 / 0.32 ms p95 per VIO message ≈ **0.5 % of one core**, GPU 0, RAM about 20 MB | measured in sim, `/team/loc/status` `proc_us_*` |
| `wall_matcher` (5 Hz, SGBM 512×320 on CPU) | about **0.6–0.8 core** (SGBM 36 ms + BEV 37 ms per frame measured on this Jetson by the prototype, plus match 40–60 ms, plus deserializing both 1024×640 images at 30 Hz); GPU 0 | prototype bench on skybeta-12; sim VM |
| organizer stack | about 1.2 cores, GPU 1–10 % | measured |

**Tonight: run only the republisher (the defaults).** The matcher would only be diagnostic with the
correction off, and it costs most of a core next to cuVSLAM. If you want its numbers, run it in a
non-scored hover: `with_matcher:=true`, still `enable_correction:=false`, and it runs `nice -n 10`.

## 4. Pre-flight check (props off)

Have a stack shell open and the topics sourced.

1. **On the pad** (default launch, drone on the launch mark facing the start heading):
   ```bash
   ros2 topic hz /team/odom_map                              # ~30 Hz (same as /visual_slam/tracking/odometry)
   ros2 topic echo --once /team/odom_map | grep -A3 "position:\|frame_id"
   ros2 topic echo --once /team/loc/status
   ros2 run tf2_ros tf2_echo course odom                     # resolves (course -> map -> odom)
   ```
   Expect:
   - frame `course` and position ≈ (-2.51, 0.91, base_height)
   - yaw = StartHeadingDeg (0°: quaternion z ≈ 0, w ≈ 1)
   - status `level 0` (OK), `anchor: live`, `anchor_tilt_source: px4_attitude`, `vo_state: 1`
2. **Carry test.** Use a separate launch, because while disarmed PX4 says "landed" and the default
   anchor would re-anchor whenever the drone is held still. Stop the default launch, then:
   ```bash
   ros2 launch team_loc team_loc.launch.py course_config:=... base_height_on_pad_m:=... ignore_land_detector:=true
   ```
   Walk the drone 1 m forward (along the start heading), 1 m left, then back onto the mark, holding
   it level at about 1 m. Expect:
   - `/team/odom_map` x goes up by about 1 m, then y up by about 1 m (course +y = left of a 0°
     heading), then it returns to within about 5 cm of the start. A sign flip or no motion means do
     not fly.
   - Status stays OK, with `anchor: frozen` after the first 10 cm.
   - Cover the camera for a second: status goes **ERROR** (`vo_state 2` or stale) and back to OK.
3. **Back to the default launch** before flying. Stop the carry-test launch, put the drone on the mark,
   start the default launch, and check step 1 again (anchor `live`, position at the mark).

## 5. Record for each run (localization topics)

Add these to the operator's bag: `/team/odom_map /team/loc/status /team/loc/correction
/visual_slam/tracking/odometry /visual_slam/status /fmu/out/vehicle_local_position
/fmu/out/vehicle_attitude /fmu/out/vehicle_land_detected /tf /tf_static`.
Bags are for people to read afterwards. Nothing loads them in a later run (rule 6.4).

## 6. If something looks wrong

| Symptom | Meaning / action |
|---|---|
| status ERROR `not anchored` | no VIO yet, or `vo_state` 2 while on the pad. Wait for cuVSLAM to track |
| position on the pad is not the launch mark | `course_config` is not the planner's config, or the drone is not on the mark. Fix before arming |
| yaw on the pad differs from StartHeadingDeg | expected only with a wrong config: the anchor uses StartHeadingDeg by definition. Place the drone at that heading |
| WARN `VIO jump absorbed` in flight | cuVSLAM teleported; the pose stayed continuous. Check the bag afterwards |
| ERROR `VIO frozen` in flight | cuVSLAM output stopped moving while the rangefinder moved. The tracker should hold, and the pilot takes over |
