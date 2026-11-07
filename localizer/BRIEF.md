# Brief: implement the `localizer` package (localization step 1)

Written 2026-11-06 at the end of the planning session. Read this whole file before you write any code.

## Goal

Estimate the drone's pose **in the course frame** (the CAD/planner map frame) well enough to fly the
precomputed path `planner/out/.../path.bin`, and stream that path as setpoints. The planner and the
localizer are separate programs. PX4's own controllers track the setpoints.

## Hard rules

- The drone has props on and a powered FC. **Never arm, and never publish to `/fmu/in/*`.** The only
  outputs to the flight stack are `/skybeta/setpoint/pose` (or `/twist`) through `flight_interface`.
  Until the bench step, publish setpoints to a preview topic only.
- Jetson: `ssh skybeta`. It is a shared account, user `dronehacks`, Tailscale `skybeta-12`.
  - Never edit other people's folders. `~/connor` is planner-only.
  - Team ROS code goes in `~/team_ws/src/localizer`. The source of truth is **this repo**
    (`C:\Users\Conno\projects\Dronehacks\localizer\`), and only what's needed gets deployed.
  - Don't change Wi-Fi, Tailscale or system settings.
  - Never `apt`/`pip install` in a running container. A team image comes from `host/run.sh build`,
    and step 1 shouldn't need one: numpy 1.26, scipy 1.15, cv2 4.5.4 (no CUDA), torch 2.5 and
    TensorRT 10.3 are already in the image.
- The user has approved Jetson actions that help (restarting the stack on the pad, recording bags,
  running passive nodes). Still announce each one. Restart the stack only while landed and disarmed:
  `docker rm -f skybeta-stack` re-zeroes the VIO, and the stack is back in about 16 s.
- The laptop runs Windows/PowerShell. Use the `py` launcher (it has numpy); `python3` there is the
  Store stub. There is no ROS on the laptop. Pure-numpy modules are tested on the laptop, and ROS
  nodes and bag replay run on the Jetson inside `skybeta-stack`.
- Prefer simplicity (see `~/.claude/CLAUDE.md`).

## What to read

On the Jetson:
- `~/skybeta/docs/competitor_guide.md` (the contract: topics, frames, envelope, arming flow).
- `~/skybeta/src/skybeta_flight_interface/` (`flight_interface.py`, `frames.py`, `envelope.py`).
- `~/skybeta/src/oakd_vslam_bringup/` (`launch/oakd_vslam.launch.py`, `px4_odom_bridge.py`).
- `~/skybeta/src/competitor_example/` (`square_mission.py`, `launch/competitor.launch.py`).
- `~/team_ws/claude/`, the earlier depth prototype: `bev.py` (BEV geometry, reuse it) and `bev_live.py`
  (`Stereo` SGBM class, `DepthAnything` TRT class, `ScaleFit`). The engines are in
  `~/team_ws/claude/models/`.
- **The measured results:** `~/team_ws/localizer_tests/` holds `drift_report.txt` (15 min stationary
  drift after a fresh restart), `bench_<ts>.txt`, `bench_<ts>/report.json` and the npz depth samples
  (SGBM vs Depth Anything cost and fill next to cuVSLAM), and `tegrastats_<ts>.log`. The bag is in
  `~/team_ws/bags/drift_still_*`. The scripts that produced them are `analyze_drift.py` and
  `bench_depth.py`, in the same folder.

In this repo:
- `planner/README.md`, for the frames and the `path.bin` format.
- `planner/config/course_v3.yaml`, for `StartPos` (-2.5146, 0.9144, 0), `StartHeadingDeg` 0 (marked
  ASSUMED), the checkpoints, and `HoverHeight` 1.0.
- `planner/maps/course_v3.bin`, a DHMAP v1 3-D point list sampled from `dronehacks-course-v3.step`.
  The course is a 6.1 x 6.1 m net cage, 3.05 m tall.
- `onboard/` and `refnav/` are **simulator** starter code that publishes `/fmu/in/*` directly. Do not use
  them on the real drone. `refnav/depth.py` is fine to read as background.

## Established facts

- Hardware: Jetson Orin Nano 8 GB (JetPack 6), Pixhawk 6C on PX4 v1.16.2, OAK-D-S2 mounted forward
  (`CAMERA_X=0.1053`, `Z=0.0304`, zero rotation in `/etc/skybeta/drone.env`), a downward rangefinder
  (max 4 m), no GPS, no optical flow.
- Stack: `skybeta` v0.3.2 in the `skybeta-stack` container, ROS 2 Humble, `ROS_DOMAIN_ID=12`, local DDS.
  - tmux session `skybeta` has windows `agent`, `slam`, `bridge` and `competitor`.
  - `slam` runs cuVSLAM 3.2 as pure stereo VO: 1024x640 @ 30 Hz, IMU fusion off, no loop closure.
  - `bridge` sends `/fmu/in/vehicle_visual_odometry` with `pose_frame=FRD`.
- **Frames.**
  - cuVSLAM `odom` is labelled ENU, but x = vehicle forward at stack start and y = left. The origin
    is the startup pose.
  - PX4's horizontal local frame follows odom: `local_enu` ≈ `odom_enu` in xy was observed.
  - Height comes from the rangefinder, not from cuVSLAM: `local_enu` z = 0.08 while odom z = 269.
  - `path.bin` is NED with its origin at the pad and x along `StartHeadingDeg`.
  - `flight_interface` converts ENU setpoints to NED by swapping x and y. `anchor_setpoints` and
    `odom_to_ned_yaw_offset_rad` are off by default; measure them on the bench (guide §1).
- **cuVSLAM gives silent wrong poses.** Before the restart, odom sat still at (19.8, 341, 286) m, then
  (24.9, 344.6, 269), with about 180° roll/pitch. It still reported `vo_state` 1 and position variance
  around 1e-9, and EKF2 followed it. Neither `vo_state` nor the covariance can detect failure.
- **The maze walls are low-texture wood with a plastic covering. Adding texture is not allowed.** Expect
  sparse stereo on wall faces (returns mostly at edges, seams and gaps), possible specular false matches,
  and weaker cuVSLAM near walls. The perimeter is net: stereo sees through it to people and the room.
- **CAD wall structure.** The 0.3–1.6 m band has 1407 vertical-face cells at 5 cm. The maze gaps
  M1/M2/M3 are 0.93 m wide and **repeat every 1.12 m in x**, so keep any matcher search window under
  about ±0.5 m or it can alias by one bay.
- **Known bugs in the prototype.**
  - `bev.py` `LocalBevMap._recentre` crashes when a jump shifts the map by at least its own size
    (`bev_live.log`). Fix: clear the map.
  - `bev_live.py` levels the camera using cuVSLAM's orientation, which is garbage when VO fails. Use
    `/fmu/out/vehicle_attitude` (IMU, gravity-referenced) for roll and pitch instead.
- Depth Anything V2-S 252x406 FP16 costs 7.9 ms of GPU per frame in trtexec, in isolation (relative and
  metric-indoor engines alike). The live measurements are in the bench results above.
- `flight_interface` sends **position-only** setpoints, so the path's velocity and acceleration
  feedforward is lost. Pose setpoints are not speed-limited. There is no horizontal geofence, and the
  altitude clamp is 0.3–3.0 m above the odom origin.
- At the end of the session the `competitor` window was running `competitor.launch.py competitor:=false`
  (`flight_interface` only, `square_mission` stopped). A stack restart brings `square_mission` back,
  because `COMPETITOR_PKG` is still `competitor_example`.

## Architecture (agreed)

Leave cuVSLAM → bridge → EKF2 untouched. Estimate one slowly varying transform **T = course←odom
(x, y, yaw)** and apply it on the **setpoint side**. Setpoint z is height above the floor, taken
straight from the path and never routed through T.

Package `localizer` (ament_python) has two nodes in separate processes, so depth work can never stall
the setpoint stream (PX4 drops out of Offboard after 1 s without setpoints):

1. **`course_matcher`**, about 5 Hz.
   - Inputs: OAK L/R images, `camera_info` and TF; `vehicle_attitude` for levelling;
     `vehicle_local_position.dist_bottom` for the height band; odometry.
   - Pipeline: depth (SGBM, or Depth Anything scaled by stereo; the source is a parameter, chosen from
     the bench results and the first course bags) → `frame_bev` → about 1 s of frames stacked using
     odom's relative motion → correlative search against a precomputed CAD likelihood field around
     the prior (±0.4 m, ±8°, coarse then fine). Ignore points outside the cage.
   - Covariance comes from the score surface, so corridor degeneracy shows up as one large eigenvalue.
   - Gates: minimum wall points, minimum score, peak vs second mode, Mahalanobis against the filter,
     `vo_state == 1`. Free-space carving through gaps counts as evidence.
   - Output: `/localizer/fix` (PoseWithCovarianceStamped, course frame, image stamp).
2. **`path_streamer`**, 20 Hz.
   - T filter: a 3-state KF with random-walk noise, updated with odom interpolated at the fix stamp.
   - Start on the pad: T₀ = pad pose ∘ odom⁻¹. Require odom near zero and odom tilt within 3° of
     PX4 attitude, otherwise refuse. Cross-check with a wide registration on the pad (±0.5 m,
     ±20°), and refuse if they disagree by more than 0.15 m or 5°.
   - Odom jump (more than 0.5 m in one step, or a `vo_state` transition): inflate T and hold the last
     safe setpoint until N consistent fixes arrive.
   - Rate-limit how fast a T correction moves the setpoint (about 0.1 m/s and 3°/s).
   - Path rows go NED-at-pad → course → odom via T⁻¹. A time-pausing carrot stops advancing when the
     tracking error is large.
   - Course-frame geofence: the cage minus a margin.
   - Outputs: `/localizer/pose` (course, 30 Hz), `/localizer/status` (JSON), and the setpoints. Publish
     setpoints to `/localizer/setpoint_preview` until the bench step.
   - Start only when `offboard_active` is true; stop advancing when Offboard ends (copy
     `square_mission.py`'s pattern).
3. **Launch file:** start `flight_interface` exactly as `competitor.launch.py` does, plus both nodes.
4. **Laptop tool `localizer/tools/cad_to_wallgrid.py`:** turn `course_v3.bin` into a 2-D wall grid at
   5 cm (band 0.3–1.6 m, keeping cells with at least 0.15 m vertical span, matching `bev.py`), plus a
   truncated distance-transform likelihood field, a cage mask and the pad pose → `.npz` and a PNG
   preview.
5. **Pre-flight survey mode** (later step): hand-carry the drone through the course, build observed
   walls in the course frame, report the per-wall offset from the CAD, and write an as-built grid
   (and planner map) when a wall is off by more than about 5 cm.

## Implementation order (each step tested before the next)

1. `cad_to_wallgrid.py` and its test: gap widths of 0.93 m, walls at CAD coordinates.
2. `match.py` (pure numpy) and unit tests on synthetic scans rendered from the grid: recover a known
   pose, large along-axis covariance in a corridor, reject a one-bay alias, stay robust to one moved
   wall, cope with sparse edge-only scans (low-texture walls).
3. Move the needed parts of `bev.py`/`bev_live.py` into the package, with the recentre fix and
   attitude-based levelling, plus a unit test for the crash case.
4. Record course bags on the Jetson with `host/run.sh bag` or a second container:
   - Include images, `camera_info`, `/tf_static`, odom, status, `vehicle_attitude` and
     `vehicle_local_position`.
   - Start on the pad after a fresh restart, hand-carry through the course pausing on taped floor
     marks at known CAD positions, include close passes along blank walls, and return to the pad.
   - The closure error measures drift and decides whether continuous correction is needed or whether
     alignment plus monitoring is enough.
5. Offline replay of `course_matcher` on those bags. Report the fix rate, the error at the taped
   marks and ms/frame, and A/B the depth sources on the same bags.
6. Pad initialisation, cross-check and status, run live but passive.
7. `path_streamer` in dry run against the preview topic.
8. T filter, rate limiting and reset handling, replayed against a bag that contains a real jump.
9. Deploy the package to `~/team_ws/src/localizer` with `team_build.sh` in the container. Ask the user
   before changing `COMPETITOR_PKG` in `/etc/skybeta/drone.env`.
10. Test ladder, each step only with the team's approval: props off on the bench (check
    `/fmu/in/trajectory_setpoint` and measure the yaw offset) → tethered hover at 0.5–1 m → pad to
    M1 at ≤1 m/s (re-plan with a lower `MaxVelMag`) → the full course with a speed ramp.

## Measured results (2026-11-07, drone stationary on a bench in a room, not in the course)

Raw files: `~/team_ws/localizer_tests/drift_report.txt`, `bench_20261107-065453.txt`,
`bench_20261107-065453/report.json` (+ npz depth samples), `tegrastats_20261107-065453.log`.
Bag: `~/team_ws/bags/drift_still_20261107-063914`.

**Stationary drift after a fresh stack restart (15 min, `vo_state` 1 throughout):**
- Over the whole 15 min: max |dxy| 3.3 cm, max |dz| 4.3 cm, max |dyaw| 0.52°. At 60 s the drift was
  only 0.4 cm. There were no steps over 5 cm.
- cuVSLAM tilt was 0.1–0.8° against a PX4 tilt of 0.35°.
- Odometry ran at 29.96 Hz by header stamps (max gap 133 ms), with track time p50 1.5 ms.
- EKF2: `xy_valid` 100%; `cs_ev_pos`, `cs_ev_yaw`, `cs_rng_hgt` and `cs_baro_hgt` all true.
- So a fresh stack on a static, textured scene is solid. The earlier 341 m garbage must come from
  something else: being handled, a person filling the view, or time. Moving drift is still
  unmeasured. Step 4 (course bags with a closure loop) is still what decides whether continuous
  correction is needed.

**Depth cost next to cuVSLAM (Python, 512x320, target 5 Hz, one process):**
| Stage | p50 / p95 ms |
|---|---|
| SGBM (CPU, 2 threads) | 36 / 49 |
| `frame_bev` (numpy) | 37–45 / 54 |
| DA-V2-S relative, TRT incl. pre/post | 11 / 15 |
| ScaleFit | 8 / 14 |
| Total, stereo only | 76 / 94 |
| Total, stereo + DA | 103 / 131 |

- cuVSLAM track time didn't change (p50 1.5–1.6 ms in every phase).
- Average CPU rose from about 22–26% to about 32–34% across 6 cores while depth ran. GPU stayed at
  3–8%. Both engines plus torch add about 230 MB of RAM.
- **The achieved rate was below 5 Hz once DA was on:** stereo only reached 5.0 Hz, stereo + DA
  3.25 Hz, and the metric engine 2.8 Hz. The likely cause is Python receiving and syncing two
  1024x640 bgr8 image streams under the GIL. Fixes: subscribe to `image_mono` instead of `image_raw`
  (a third of the bytes), and/or skip frames before deserialising. `frame_bev` is as expensive as
  SGBM, so vectorise or cut its free-space carving.
- Fill on this room scene: stereo 53%, DA 100% when the fit succeeds.
- **The metric-indoor engine fails the existing `ScaleFit`** (DA fill p50 0). It outputs depth, not
  inverse depth, so the 1/z = s·rel + t fit doesn't apply and would need its own z = s·d + t fit.
- Odometry arrival latency, measured inside the benchmark process, was noisy (p50 60–163 ms with no
  consistent trend between phases), so it is contaminated by the benchmark's own load. Re-measure
  EKF2-relevant latency with `~/skybeta/tools/latency_probe.py --seconds 30` with and without
  `course_matcher` running in its own process.

## Open items

- `StartHeadingDeg` is assumed to be 0. Confirm it on the real course.
- The material and texture of the nets and floor are unknown. Check them in the first bag.
