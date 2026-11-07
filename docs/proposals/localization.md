# Localization proposal (drone 12)

Status: **proposal for review**. Nothing is built yet. Branch `localization`, cut from `sim-and-plans`.
Written 2026-11-07, about 02:30 PST. Scored runs start Sat 22:00 and code is due Sun 16:00, so this
plan has roughly 19 working hours before the first scored run.

## 0. Summary

- **Milestone 1 is a republisher.** It anchors the correction on the launch pad from the course
  config, then republishes every cuVSLAM message in the course frame with the latest correction.
  It also publishes an honest status. A C++ node, small, ready Saturday morning.
- **First drift correction: 2D wall matching.** Stereo depth (OpenCV SGBM at half resolution on the
  CPU) is levelled with PX4's attitude, sliced to a height band, stacked over about 1 s, and matched
  against a likelihood field rasterized from `planner/maps/course_v3.bin`. A correlative search
  around the prior does the matching. A 4-state EKF on the `course → odom` correction
  (x, y, z, yaw) fuses the fixes. Height comes from the PX4 rangefinder. A team prototype of the
  depth, BEV and matcher parts already runs on drone 12 at 5 Hz next to cuVSLAM; we port it rather
  than start over.
- **Second method (Sunday, if time allows): 3D registration** of the same depth cloud against the
  course point cloud with small_gicp. It cross-checks the 2D fixes and covers the return flight
  above the walls, where a wall slice sees nothing.
- **Not this event:** YOLO landmarks, GTSAM, ESS, on-camera depth (that needs the organizers),
  and a replacement VIO.
- **Contract change needed (blocking, small):** cuVSLAM already publishes TF `map → odom`. We
  propose that the course frame is named `course` in TF and messages, and that localization
  publishes TF `course → map` (section 2).

## 1. What I found

### 1.1 On the drone (skybeta-12, stack 0.3.2, read-only inspection)

| Finding | Consequence for us |
|---|---|
| cuVSLAM (`isaac_ros_visual_slam` 3.2.6) runs with `publish_map_to_odom_tf: True`, `map_frame: map`, `enable_localization_n_mapping: False`. Live `/tf` has `map → odom` (from `visual_slam_node`) and `odom → base_link`. | **We cannot also publish `map → odom`.** Two parents or two publishers of one edge break TF. See section 2. |
| `enable_imu_fusion: False`; `enable_image_denoising: False`; `rectified_images: False`. Odometry ≈ 30.5 Hz; track time ≈ 1.7 ms mean, 4.8 ms max. | cuVSLAM is light. Pure stereo VO, so there is no gravity in `odom`: roll and pitch of `odom` are those of `base_link` at stack start. |
| `/visual_slam/status.vo_state`: 0 unknown, 1 tracking, 2 lost. | Primary input to our status. |
| Odometry pose covariance ≈ 1e-8 (meaningless). Twist is body FLU (competitor guide). | We fill our own covariance. Twist passes through unchanged. |
| `odom` origin = `base_link` where the stack started. The stack starts at boot, and the drone may be carried to the pad later. | Anchor on the pad from the current odom pose, not at the odom origin. Path following's fallback already does this. |
| `/oak/{left,right}/camera_info`: 1024×640, `rational_polynomial` with **all-zero distortion**, fx ≈ fy ≈ 640, cx ≈ 509, cy ≈ 309. Stereo baseline from TF ≈ 7.56 cm. | Rectification uses D = 0. If the lens really distorts, depth is biased toward the image edges. Check it on the bench: vertical disparity residual on a checkerboard, or straight edges. |
| No depth topic. The camera runs at 118 of about 143 MB/s USB because the driver ships mono as bgr8 (organizer measurements in `oakd_vslam.yaml`). | On-camera depth would need an organizer config change and the USB headroom is small. We compute depth on the Jetson. |
| PX4 topics available read-only: `/fmu/out/vehicle_attitude` (IMU attitude), `/fmu/out/vehicle_local_position` (`dist_bottom` from the rangefinder; `EKF2_HGT_REF` = rangefinder), `/fmu/out/vehicle_land_detected`. | Levelling the depth, z correction, and "on the pad" detection come from PX4, not cuVSLAM. |
| Load with the stack running (plus teammates' dashboard and MAVLink relay): **CPU ≈ 30–40 % of 6 cores at 1.73 GHz, GPU (GR3D) 1–10 %, RAM 3.3 / 7.6 GB**, MAXN_SUPER. The stack alone is about 1.2 cores: camera driver 0.47, cuVSLAM container 0.35, flight interface 0.18, odom bridge, agent and status about 0.2. | About 3 free cores and an almost idle GPU. Budget in section 5. |
| An earlier team prototype is already on the drone: `~/team_ws/src/localizer` (`depth.py` SGBM, `bev.py` levelled BEV, `match.py` correlative matcher, `course_matcher` node, `path_streamer`) plus `~/team_ws/claude/` (Depth Anything V2-S TensorRT engines, a basalt VIO test, a dashboard). Its own notes (`HALF_ASSED.md`): SGBM at 512×320, 96 disparities, **36 ms p50, 5 Hz next to cuVSLAM**; Depth Anything only reached 3.3 Hz; matcher thresholds are tuned on synthetic scans only; no fusion. It is not in this repo. | We port `depth.py`, `bev.py` and `match.py` into this repo in milestone 3. We drop `path_streamer`, which is path following's job. |
| The IMU calibration probe (`imu_calib.json`) found the driver's `oak_imu_frame` TF off by about 92°. | One more reason not to use the OAK IMU (OpenVINS etc.) without our own extrinsic. |
| `~/skybeta/docs/` is deleted in the drone's working tree. The guide is still in git (`git show HEAD:docs/competitor_guide.md`). | Read it from git; don't restore files in `~/skybeta`. |

### 1.2 The map

`course_v3.bin` has 301,963 surface points. The interior objects, read off a height map of the points:

- **Walls are 1.70 m tall everywhere** (68 in): the maze (a long east-west wall plus three dividers
  in the north half), the chicane walls C1, C2a (with the 0.91 m window), C2b, and the A1
  approach wall. At any flight height up to about 1.6 m, a horizontal slice hits every wall. This is
  what makes 2D matching attractive.
- **Tube:** an oval 1.47 × 1.83 m inside and 1.52 m long, axis along y, on cradles. Inside it the
  slice sees two curved sides: a short corridor of about 1.5 m with no constraint along its axis.
  VIO drift over 1.5 m is a few cm, so that is acceptable.
- **Boulders** (0.2–0.5 m) and **barriers** (low). These are below the slice, so only 3D registration uses them.
- **Nets** on the four sides and the roof. Stereo sees through mesh, so net returns are unreliable
  and the room beyond is not in the map. The matcher must ignore everything at or beyond the
  cage (the prototype does this with a cage mask: the likelihood is constant outside it).
- **Landing pad, launch mark, guideline:** floor markings only. They are invisible to depth, and only a
  camera detector could use them.
- **Aliasing:** maze bays repeat at about 1.12 m. Any search window has to stay under about ±0.5 m,
  and the prior has to be good. That is why the pad anchor matters.

### 1.3 Research

Numbers with sources are in section 9. In short:

- **2D walls.** Correlative scan matching (Olson 2009; the real-time matcher inside Cartographer)
  scores poses against a likelihood field by brute force. It is deterministic and gives a
  covariance from the score surface, and a narrow-FOV, noisy stereo "scan" suits it. `nav2_amcl`
  uses the same likelihood-field model in a particle filter. AMCL needs a `LaserScan` and a
  levelled TF frame, broadcasts `map → odom` by default, and updates on motion thresholds. That is
  more integration for the same model. It is our fallback if the correlative matcher misbehaves.
  `depthimage_to_laserscan` takes rows of the unlevelled image. At 10–15° of forward pitch those
  rays hit the floor at 4–6 m, so we level with PX4's attitude ourselves.
- **3D registration.** small_gicp is fast on CPU (section 9). The target KD-tree is built once from
  the course cloud, and a few thousand downsampled depth points register in milliseconds.
  `small_gicp_relocalization` is the same pattern but publishes `map → odom` directly, with no
  gating or fusion. We use the library, not the node.
- **Visual landmarks (YOLO, arXiv 2512.20475).** See section 9: that paper trained on real
  labelled images, not renders, and fused PnP positions into a linear drift KF. For us the landmarks are few (window, tube mouth, pad, wall ends), the sim textures
  are stand-ins, and a detector needs renders, training, a TensorRT export and a filter
  measurement model. That is more than our remaining time, and depth already covers walls and the window.
  Worth it after the event, or for the landing pad (floor markings depth cannot see).
- **Fusion.** robot_localization's EKF estimates the robot's state and publishes its own
  `map → odom` (it can be turned off). It outputs at a fixed rate, not per VIO message, and needs
  covariance tuning across 15 states. GTSAM iSAM2 would be right for many landmark factors and
  smoothing, but the setup cost is too high for this time budget. A 4-state EKF on the correction
  is about 150 lines, matches the contract's "republish every VIO message with the latest
  correction", and is the standard 4-DOF drift correction in VIO loop closure (VINS-Mono's pose
  graph is 4-DOF for the same reason: roll and pitch are observable from gravity, x, y, z and yaw are not).
- **Depth on the Jetson.** SGBM at half resolution is measured on this drone (36 ms p50). VPI's CUDA
  SGM (13 ms at 960×540 measured on an Orin Nano Super) would move it to the idle GPU if the CPU
  gets tight. ESS is heavier and needs Isaac ROS
  packages we would have to add to the image. On-camera depth is the organizers' config and is
  short of USB bandwidth. Details in section 9.
- **Dim tunnel.** cuVSLAM needs texture and stable brightness. The failure risks are auto-exposure
  lag at the tube mouth, motion blur from longer exposure, and a textureless interior. We control
  none of the camera settings (the organizers' driver config). What we can do: measure it in sim
  with a darker tube; make status go WARN on falling feature quality before `vo_state` = 2; ask
  path following to fly the tube straight, slowly and facing the exit; and ask the organizers
  about exposure and lighting (section 7).

## 2. Frames and the contract change

cuVSLAM owns `map → odom → base_link`. Its `map` is not the planner's course frame: with
localization and mapping off it should be identity, so `map ≈ odom`. Proposal:

| Frame | Meaning |
|---|---|
| `course` | **The planner's frame** (what CONTRACTS.md calls `map`): metres, z up, origin at the cage-floor centre, launch mark ≈ (-2.51, 0.91). |
| `map` | cuVSLAM's, identity to `odom` (we never assume this; we read it from TF). |
| `odom`, `base_link` | Unchanged. |

- Localization publishes **TF `course → map`** = `T_course_odom · T_map_odom⁻¹`, read live from
  cuVSLAM's TF. It is correct even if cuVSLAM's `map → odom` ever stops being identity.
- `/team/odom_map`: `header.frame_id = course`, `child_frame_id = base_link`.
- Path following looks up `course → odom` instead of `map → odom`. TF composes our edge with
  cuVSLAM's. One frame string changes in their code (`frames.py`).
- Alternatives rejected: asking the organizers to set `publish_map_to_odom_tf: False` would change
  their stack; publishing no TF would lose Foxglove and RViz views and make the topic the only path.

**Needs:** a CONTRACTS.md edit agreed by both sessions. Until then, path following should use the
`/team/odom_map` topic, which carries the correction already, rather than the TF.

### Initialisation (the pad anchor)

The correction is anchored while PX4 reports landed, the drone is still, and `vo_state` = 1:

```
T_course_base(launch) = (StartPos + (0, 0, h_base), Rz(StartHeadingDeg) · R_rp)
T_course_odom         = T_course_base(launch) · T_odom_base(now)⁻¹
```

- `StartPos` and `StartHeadingDeg` come from the course config (path given as a parameter).
- `R_rp` is roll and pitch from `/fmu/out/vehicle_attitude` (gravity), so a stack that started tilted still gives a level `course` frame.
- `h_base` is the height of `base_link` (the Pixhawk IMU) above the floor on the pad. It is a
  vehicle parameter, measured once with a tape. It is not a course number.
- The anchor is refreshed continuously while landed (so carrying the drone or VIO creep before
  takeoff does no harm) and frozen at takeoff (`landed` false or z rising).
- From milestone 3 on, a **wide match on the pad** (±0.5 m, ±20°) checks the anchor against the
  walls visible from the launch mark. It catches a wrong `StartHeadingDeg` (still marked ASSUMED
  in the config) or a misplaced drone. Beyond 0.15 m or 5° it sets WARN and logs the measured
  offset. It never re-anchors silently.

Answers to path following's questions: `h_base` is a launch parameter of our node (default from
the vehicle config, not the course config). Added latency is under 1 ms (section 5). The correction
never steps: the applied correction slews at most **0.20 m/s and 5°/s**, so a fix that moves the
estimate by 0.2 m shows up over 1 s. Those are parameters; use 0.3 m/s as the test value for T7.

## 3. Architecture

```
/visual_slam/tracking/odometry ─┬──────────────────────────────────────────▶ loc_fuser (C++) ──▶ /team/odom_map   (every VIO msg)
/visual_slam/status ────────────┤                                               │  ▲            ──▶ TF course → map
/fmu/out/vehicle_attitude ──────┤                                               │  │            ──▶ /team/loc/status (10 Hz + on change)
/fmu/out/vehicle_local_position ┤  (dist_bottom → z fix)                        │  │            ──▶ /team/loc/correction (debug)
/fmu/out/vehicle_land_detected ─┘  (anchor while landed)                        │  │
                                                                                 │  │ /team/loc/fix
/oak/{left,right}/image_mono + camera_info ──▶ wall_matcher (Python, ~5 Hz) ◀───┘  │ (PoseWithCovarianceStamped,
TF base_link → optical frames                    SGBM → level → band → stack → match ─┘  course frame, image stamp)
                                              ──▶ /team/loc/matcher_status (JSON debug)
(milestone 5) cloud_matcher: same depth → small_gicp vs course cloud ──▶ /team/loc/fix (same type, tagged by source)
```

### 3.1 `team_loc` (ament_cmake, C++): node `loc_fuser`

The hot path, so C++: it sits in path following's control loop, and rclpy's executor jitter
under load is the wrong risk to take for a 30 Hz pass-through.

- **On every VIO message:** compose `T_course_odom_applied · T_odom_base`. Publish
  `/team/odom_map` with the VIO stamp, pose in `course`, twist copied (body FLU). Covariance =
  correction covariance (x, y, z, yaw, mapped to 6×6) plus a VIO term growing with distance since
  the last accepted fix. Publish TF `course → map` with the same stamp. No queueing: KEEP_LAST 1.
  Matching the publisher's RELIABLE is fine.
- **Estimator:** EKF state `[tx, ty, tz, ψ]` of `T_course_odom` (the 4-DOF left correction).
  - Process noise grows with odom path length and with time: `Q = (σ_d² · |Δp| + σ_t² · Δt) · I`.
  - A fix (x, y, ψ of `base_link` in `course` at the image stamp) is predicted from the state and
    the odom pose **interpolated at the image stamp** from a 2 s ring buffer. The fix's covariance
    comes from the matcher's score surface.
  - Mahalanobis gate χ²₃ at 99 %. Three consecutive rejected fixes that agree with each other mean
    re-acquire: inflate P and accept the next consistent fix.
  - z fix: `floor_z(x, y)` from the map plus `dist_bottom`, only when PX4 marks it valid and the map
    says plain floor below (not a boulder or barrier).
- **Applied vs estimated correction:** the applied one slews toward the estimate (0.20 m/s,
  5°/s). The residual is added to the published covariance.
- **Odom jump guard:** a VIO step above a speed bound, or `vo_state` going 2 → 1, means cuVSLAM
  reset or teleported. Then set ERROR, freeze the published pose source, inflate P, and re-acquire
  from fixes. (PX4 has likely left Offboard by then anyway; the point is never to publish a
  confident wrong pose.)
- **`enable_correction` parameter:** false = milestone-1 behaviour (anchor only). This is the
  safety valve if the correction is not proven by Saturday evening.

### 3.2 `/team/loc/status` (DiagnosticStatus, name `team_loc`)

| Level | When (any) |
|---|---|
| `ERROR` (2) | `vo_state` = 2; VIO older than 0.2 s; odom jump; not anchored; position σ > 0.5 m |
| `WARN` (1) | VIO rate < 20 Hz over 1 s; no accepted fix for > 5 s while moving (from M3); position σ > 0.15 m or yaw σ > 5°; correction still slewing > 0.10 m; pad wide match disagrees with the anchor; > 3 fixes rejected in a row |
| `OK` (0) | otherwise |

`values` carry `vo_state`, VIO rate and age, σ_xy, σ_yaw, last fix age, fix source, rejected
count, and the applied-vs-estimated residual, so Foxglove shows the reason.

### 3.3 `team_loc_matcher` (ament_python): node `wall_matcher`

Ported from the drone prototype (`depth.py`, `bev.py`, `match.py`), with these changes:

- Topics, image size and band all come from parameters. Sim (1280×800 pinhole) and drone
  (1024×640) differ.
- The prior is `/team/odom_map` at the image stamp, not a private pose.
- Output is `/team/loc/fix` for the fuser to gate. No filter inside the matcher.
- **Height band referenced to the floor:** `dist_bottom` + attitude give the camera height, and
  the band is `[0.3, min(wall top) − 0.1]` m above the floor. That is read from the derived map's
  metadata, not hard-coded.
- **Degeneracy:** the covariance's long axis shows corridors; the fuser takes it as is.
- Runs at `nice 10`, at most 2 OpenCV threads, processing only the newest pair (5 Hz); older ones
  are dropped.

`tools/map_to_wallgrid.py` (new, derived map; never hand-edited) reads any DHMAP file and
course config. It rasterizes the points within a height band into a 2D likelihood field
(5 cm, Gaussian of distance-to-wall), plus a "known area" mask. Points on the transparent cage
boundary are excluded via a CLI option that names the boundary, so no course numbers live in
code. It writes `.npz` with its own metadata (band, wall-top height, resolution, source map hash).

### 3.4 Libraries

| Library | Use | Why |
|---|---|---|
| ROS 2 Humble `rclcpp`, `tf2_ros`, `diagnostic_msgs`, Eigen | fuser | In the image already |
| `px4_msgs` (image's), `isaac_ros_visual_slam_interfaces` | inputs | Must match the FC; already in the image |
| OpenCV `StereoSGBM`, `stereoRectify` | depth | Measured on this drone; CPU, no new deps |
| NumPy | BEV and correlative matcher | Prototype; pure, tested |
| small_gicp (pip / source, MIT) | milestone 5, 3D registration | Fastest open GICP on CPU, Python bindings |
| NVIDIA VPI (in JetPack) | only if SGBM's CPU cost is too high | CUDA stereo on an idle GPU |

All of these get credited in the top-level README (rule 9.1).

## 4. Rules this design respects

- Only map data from the published layout: `course_v3.bin` and the grids derived from it by
  script. **No node writes anything it later reads.** No cuVSLAM map, no saved scans, no learned
  offsets between runs (rule 6.4). Bags recorded on the real course are for humans to look at
  only. They never feed parameters automatically, and never get committed or loaded by a node.
- No course numbers in code: start pose, heading, map, band and boundary come from the config
  and map files.
- Never publishes to `/fmu/in/*` or `/skybeta/setpoint/*`. It only reads `/fmu/out/*`. It never
  touches `~/skybeta`, PX4 parameters or Tailscale.

## 5. Compute budget (Orin Nano 8 GB, MAXN_SUPER, 6 × A78AE)

| Process | CPU (cores) | GPU | Notes |
|---|---|---|---|
| Organizer stack (measured) | ~1.2 | 1–10 % | driver 0.47, cuVSLAM 0.35, flight interface 0.18, rest 0.2 |
| `loc_fuser` (C++) | < 0.03 | 0 | 30 Hz pass-through plus a 4-state EKF; added latency target < 1 ms p95 |
| `wall_matcher`: image subscription | ~0.15 | 0 | rclpy deserializes 2 × 0.65 MB at 30 Hz even when we drop most; fallback is a C++ throttle node |
| `wall_matcher`: SGBM 512×320 @ 5 Hz | ~0.2 | 0 | 36 ms p50 measured on the drone, 2 threads |
| `wall_matcher`: level, BEV, stack, match @ 5 Hz | ~0.2 | 0 | to profile; cap the search lattice to fit 40 ms |
| `cloud_matcher` small_gicp @ 2–5 Hz (M5) | ~0.1 | 0 | about 5k source points, target tree prebuilt |
| Path following (their estimate) | ~0.2 | 0 | |
| **Total** | **~2.1 of 6** | **≤ 10 %** | leaves ≥ 3 cores; cuVSLAM untouched |

Rules: our matcher runs at `nice 10` and drops frames rather than queueing. Bag recording during
flights uses the competitor guide's topic set, no full-rate images on scored runs. If cuVSLAM's
`track_execution_time_max` or the VIO rate degrades with our nodes up, the matcher rate halves
automatically (a parameter, measured in M4).

## 6. Evaluation (sim slot 1, `ROS_DOMAIN_ID` 11)

Ground truth `/sim/ground_truth/odom` (100 Hz) is already in the planner/course frame (the world
frame is the map frame).

**Record:** `/visual_slam/tracking/odometry`, `/visual_slam/status`, `/team/odom_map`,
`/team/loc/status`, `/team/loc/fix`, `/team/loc/matcher_status`, `/sim/ground_truth/odom`, `/tf`,
`/tf_static`, `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_attitude`, and the camera pair
plus camera info, so the matcher can be replayed offline from sim bags.

**`tools/loc_eval.py` (milestone 2)**, from a bag, no alignment fitting (absolute error in `course` is what flies):

- position and yaw error vs time, for raw VIO (anchor only) and for corrected output
- ATE RMS and max over the course; error at each checkpoint (closest approach to the box centre),
  at the pad, and back at launch
- drift per metre travelled (raw VIO)
- **status honesty:** fraction of time with error > 0.30 m while OK (target 0), and lead time from
  WARN to error crossing 0.30 m
- **latency:** receive time of `/team/odom_map` minus receive time of the VIO message with the same stamp (p50, p95, max)
- fix statistics: accepted and rejected count, innovation vs GT error

Results go in `docs/results/localization.md`, one row per run (commit, scenario, method, numbers).

**Scenarios** (each at least 3 runs, because the RRT path and the VIO vary):

1. Nominal full course at the default speed, and at 1.5× speed.
2. Dim tube: lower the tube's lighting and texture in the world file. Find where cuVSLAM degrades
   and check WARN comes before the error grows.
3. Forced dropout: stop the camera bridge for 0.5 s and for 2 s mid-course. Expect ERROR within
   0.2 s, no confident wrong pose, and recovery by re-acquire if VIO comes back.
4. Bad anchor: start 0.3 m off and 10° rotated. The pad wide match flags it.
5. Sim-to-real stress: fewer textures on walls; net returns (add clutter outside the cage); an
   as-built offset (move one wall 5 cm). The gating must hold.

**Pass bar for flying the correction** (else fly `enable_correction:=false`): it beats raw VIO on
ATE and at the pad in all nominal runs, never ERROR in nominal runs, honesty target met, and
added latency p95 < 2 ms.

**On the drone (props off, hand-carried, nothing saved for later runs):** fuser latency and CPU
with the full stack; matcher rate; pad wide match from the real launch mark; a walk through the
course (if the organizers allow it) to watch fixes live.

## 7. Risks

| Risk | Mitigation |
|---|---|
| A wrong correction is worse than none | Gating, slew limit, the `enable_correction` switch, a pass bar before flying it, and status WARN when the correction disagrees with VIO |
| Real walls have little texture, so SGBM gives holes, no fixes, and possibly weak cuVSLAM | Free-space evidence and stacking over 1 s; fixes are optional (status WARN, fall back to VIO); ask the organizers about materials; test with flat textures in sim |
| Nets and the room beyond produce stray depth | Likelihood constant outside the known-area mask; range cap; robust per-point floor |
| Maze aliasing (bays every 1.12 m) | ±0.4 m window around a good prior; the gate; a second mode must be separated by a valley or the fix is rejected |
| Zero distortion in `camera_info` is wrong | Bench check (vertical disparity); if needed, calibrate on the bench (lessons, not course data) and pass D as a parameter |
| The return above the walls has no slice | Coast on VIO plus the last correction (drift over about 6 m is a few cm); status reflects fix age; small_gicp in M5 sees wall tops and floor |
| CPU contention slows cuVSLAM, VIO drops, the run ends | nice 10, frame dropping, the M4 profile, the matcher halves its rate automatically |
| The TF contract change is not agreed in time | Path following uses `/team/odom_map` (topic), which needs no TF |
| `StartHeadingDeg` is still ASSUMED in the config | Pad wide match reports the measured heading; ask the organizers |
| Sim differs from the drone (cameras, textures, airframe) | Parameters for every camera value; drone bench test before net time |
| Time: about 19 h to the first scored run | Milestones ordered so each one is flyable on its own; decision gate at Sat 18:00 |

## 8. Questions for the organizers

1. **Depth:** can the stack publish OAK-D on-camera stereo depth (a `StereoDepth` node in
   `oakd_vslam.yaml`, e.g. 640×400 at 15 Hz)? Or is computing it on the Jetson the expected route?
2. **Camera exposure:** what auto-exposure limits are configured (max exposure time, ISO)? May
   teams set runtime exposure parameters on `/oak`, or is that part of the stack we must not change?
3. **Tunnel lighting:** is the tube lit at all, and how dark is it compared with the rest of the
   cage? Is there texture inside?
4. **Materials:** what are the maze and chicane walls made of, and in what colour or texture? Is
   the cage net dark mesh?
5. **Launch:** is the drone placed on the launch mark at a fixed heading (which one)? Is the
   stack restarted on the pad before each run, or does it keep running from boot?
6. **As-built:** how closely does the course match `dronehacks-course-v3.step`? Will parts move
   between runs? May we walk the drone through the course with live, unrecorded localization before our slot?
7. **Frames:** is cuVSLAM's `map → odom` TF meant for teams? Any objection to a team publishing
   TF `course → map` above it?
8. **Checkpoints:** are the official checkpoint lines the ones in our config (M1–M3, tube, C2a window)?

## 9. Research notes and sources

"Estimate" marks our own arithmetic; everything else is from the cited source.

**Drift-corrected VIO for racing (arXiv 2512.20475, KAIST, A2RL).**
- VIO and detector: monocular OpenVINS. YOLOv8s-Pose (640², TensorRT FP16) detects gate boxes
  plus the 4 inner corners; about 16 ms per frame on an Orin NX.
- Measurements: detections are matched to the known gate map by reprojection (Hungarian), then
  `solvePnP` IPPE_SQUARE gives the camera position (rotation discarded as too noisy). Gates:
  1–13 m, skew ≤ 2, an occlusion filter.
- Fusion: a loosely coupled **linear KF on the VIO drift** (position plus drift velocity).
  Measurement covariance comes from perturbing the detections 50 times.
- Training: on about 2,000 **real hand-labelled images**, not renders; synthetic data is listed as future work.
- Result: ATE 1.04 → 0.56 m. Drift builds up between landmarks, and short exposure (against blur)
  darkened the images and hurt the detector.
- Takeaway for us: the loose drift-KF structure is what section 3.1 does, with walls instead of
  gates. A detector trained on renders alone is less proven than the task brief assumed.
- https://arxiv.org/abs/2512.20475, https://arxiv.org/html/2512.20475v1

**Stereo depth on the Orin Nano.**
- **VPI SGM (Isaac ROS `DisparityNode`, CUDA):** measured on Orin Nano Super, 960×540, 64
  disparities: **84 fps, 13 ms**, GPU about 92 % at saturation. At 512×320 and 5 Hz that is an
  estimated few percent of the GPU. This is the best upgrade path if SGBM's CPU cost matters.
  The Orin Nano has no PVA or DLA, and an OFA is unconfirmed, so use the CUDA backend.
  isaac_ros_benchmark `isaac_ros_disparity_node-orin_nano.json`;
  https://docs.nvidia.com/vpi/3.2/algo_stereo_disparity.html
- **ESS:** Humble packages exist for Isaac ROS 3.2. Orin Nano Super numbers exist only in the 5.0
  docs: light ESS (480×288) 82 fps / 13 ms, full 35 fps / 30 ms. It needs the Isaac ROS ESS
  packages and models in our image, so it is heavier to deploy than VPI.
  https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_ess/index.html
- **OpenCV SGBM:** no published Orin number. On this drone the prototype measured 36 ms p50 at
  512×320 with 96 disparities next to cuVSLAM, the only number that counts.
- **On-camera (RVC2):** `pipeline_gen.i_pipeline_type: RGBD` (or `Depth`) with `stereo.*` params.
  Fast Accuracy reaches 90 fps at 400P. Depth is aligned to the RGB camera unless
  `stereo.i_aligned: false`. 640×400 16-bit at 15 Hz adds about 7.7 MB/s (estimate). The organizers
  measured the link saturating around 143 MB/s with today's 118 MB/s of bgr8 monos, so the margin
  is thin, and it is their config either way. https://docs.luxonis.com/software-v3/depthai/ros/parameters,
  https://docs.luxonis.com/overview/toplevel-features/depth/

**small_gicp.** The only published timings are KITTI frame-to-frame on a 64-core desktop: GICP
about 60 ms with 1 thread and 24 ms with 4, including KD-tree and covariance builds. There are no
ARM numbers. For scan-to-map the target tree is built once; with a few thousand source points,
estimate tens of ms on 2–4 A78 cores, to be measured. `small_gicp_relocalization` expects an
odom-registered cloud (FAST-LIO style), runs GICP at 2 Hz, publishes only TF `map → odom`, and has
no fitness gating or covariance. Use the library, not the node.
https://github.com/koide3/small_gicp/blob/master/BENCHMARK.md,
https://github.com/Alex10086/small_gicp_relocalization

**nav2_amcl / depthimage_to_laserscan.** AMCL takes any `LaserScan`, publishes `amcl_pose` and TF
`map → odom` (`tf_broadcast: false` turns it off), needs odom → base TF, and does not update while
hovering under its defaults (`update_min_d` 0.25 m, `update_min_a` 0.2 rad).
`depthimage_to_laserscan` assumes a level camera. https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp,
https://github.com/ros-perception/depthimage_to_laserscan/tree/ros2

**robot_localization.** `publish_tf: false` works and it still publishes `odometry/filtered`.
Fusing VO as `odom0_differential: true` plus absolute poses as `pose0` with `world_frame: map`
works, with Mahalanobis rejection. https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml

**GTSAM.** `ros-humble-gtsam` 4.2.0 exists for jammy arm64. iSAM2 averages 2–30 ms per step on
1,000–3,500-pose graphs (Kaess 2012), which is fine for us, but the build-out time is not.

**cuVSLAM in low light.**
- No documented lux or texture minimum. NVIDIA attributes tracking loss to blur, low light, or
  wrong `camera_info`. `enable_image_denoising` is recommended for noisy low-light images, with no
  numbers. cuVSLAM falls back to IMU for under a second only when IMU fusion is on, and **it is
  off on our stack, so there is no fallback**.
- depthai exposure: `r_set_autoexposure_limit` with `r_auto_exposure_limit` (µs, documented default
  1000). If that default is live, the tube will be dark and noisy rather than blurred; ask the
  organizers (Q2).
- Blur estimate: about 640 px/rad × ω × t_exp, so at 1 rad/s a 5 ms exposure smears about 3 px.
- https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/visual_slam/cuvslam/index.html

## 10. Milestones (times PST)

| # | By | Deliverable | Done when |
|---|---|---|---|
| **M1** | Sat 07:00 | `team_loc/loc_fuser` republisher: pad anchor from the course config and PX4 attitude/landed, `/team/odom_map` on every VIO message, TF `course → map`, `/team/loc/status` (vo_state, rate, staleness, anchored), `enable_correction` stub. Unit tests for the anchor math and status levels. | Runs in sim slot 1; `/team/odom_map` matches GT at the pad within 2 cm at start; added latency p95 < 1 ms; path following switched to it |
| **M2** | Sat 09:30 | `tools/loc_eval.py` + bag record script; raw-VIO baseline on the full course in sim (3 runs) in `docs/results/localization.md` | Table shows drift per metre, error at every checkpoint, pad, launch |
| **M3** | Sat 15:00 | `team_loc_matcher/wall_matcher` ported from the prototype, `tools/map_to_wallgrid.py`, EKF fix fusion with gating and slew, rangefinder z, pad wide match | Sim nominal: corrected beats raw VIO on ATE and at the pad; no ERROR in nominal runs |
| **M4** | Sat 18:00 | On the drone, props off: CPU/latency profile with the full stack, matcher rate, pad wide match from the real launch mark, camera distortion check. **Decision gate:** fly with correction on or off at 22:00 | Numbers in the results table; rate auto-halving tested |
| **M6a** | Sat 20:00 | Dropout and bad-anchor scenarios in sim (scenarios 3, 4) | ERROR within 0.2 s; bad anchor flagged |
| — | Sat 22:00 | **Scored runs** with whatever passed the gate | |
| **M5** | Sun 10:00 | `cloud_matcher` (small_gicp vs course cloud) on the same depth, compared with the wall matcher in sim; keep the better one, or fuse both | Results table row; return-leg error reduced |
| **M6b** | Sun 11:30 | Dim-tube scenario and texture stress (scenarios 2, 5); status thresholds tuned from sim | Honesty target met |
| — | Sun 12:30 | Repo link due; README credits for every library | |
| — | Sun 16:00 | Final code pushed | |

Explicitly out of scope unless everything above is done: YOLO landmarks, VPI or ESS depth (unless
M4 shows SGBM is over budget), GTSAM, an alternative VIO, and robot_localization as a comparison.
