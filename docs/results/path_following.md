# Path following results

Metrics (same in every sim): **mission** = Offboard start to touchdown; **course** = path clock start to
landing start; **plan** = planned duration; **max/rms xt** = distance of the true position from the
plan; **min clear** = true position to nearest map point (map 5 cm grid); **touches** = entries within
0.20 m of an obstacle (body + props); **checkpoints** = planner boxes entered in order (strict: the boxes
are ±0.08–0.25 m inside ~1 m openings); **cp miss** = worst distance outside a box; **land err** = touchdown
to goal xy.

Tracker: `ff_mode: inverse` (v_ref + τ_v a_ref at t + t_delay), τ_v 0.55, t_delay 0.10, Kc 1.2, Ka 0.6, Kz 1.5,
cross-track-priority saturation, time-scaled clock (e0 0.10 m, e_span 0.30 m). Defaults in
`path_following/team_path_following/config/tracker.yaml`.

## Offline: kinematic sim (`path_following/tools/kinsim.py`)

Model: flight interface (latest twist, 3D norm clamp, zero-order hold at `setpoint_rate_hz`, 1 s timeout),
PX4 velocity PID from the fleet parameter files (XY P/I/D 1.8/0.4/0.2, Z 4/2), 45° tilt limit, 80 ms
attitude lag, linear drag, 3 s takeoff ramp, auto-disarm; VIO 30 Hz, 60 ms latency, 1 cm noise. Two
noise seeds per row unless noted. Not Gazebo, not the real airframe.

### Course v3, plan MaxVelMag 1.2 (17.03 s), cap 1.5 m/s

Feedforward is what matters; setpoint rate barely does:

| feedforward | Kc | rate Hz | course s | max xt | min clear | touches |
|---|---|---|---|---|---|---|
| none (v_ref only) | 1.2 | 10 | 30.5 | 0.358 | 0.10 | 8 |
| look-ahead 0.3 s | 1.2 | 10 | 23.7 | 0.216 | 0.15 | 2 |
| look-ahead 0.6 s | 1.2 | 10 | 19.9 | 0.165 | 0.25 | 0 |
| look-ahead 0.6 s | 1.2 | 50 | 19.7 | 0.176 | 0.22 | 0 |
| look-ahead 0.9 s | 1.2 | 10 | 22.3 | 0.285 | 0.14 | 9 |
| **inverse τ 0.55, d 0.10** | 1.2 | 10 | **17.6** | **0.124** | **0.30** (= plan) | 0 |
| inverse τ 0.40, d 0.15 | 1.2 | 10 | 18.7 | 0.116 | 0.25 | 0 |
| inverse τ 0.70, d 0.05 | 1.2 | 10 | 17.6 | 0.144 | 0.30 | 0 |

Robustness of the default against plant mismatch:

| mismatch | course s | max xt | min clear | touches |
|---|---|---|---|---|
| none | 17.6 | 0.124 | 0.30 | 0 |
| PX4 XY P 1.2 (slower loop, τ≈0.89 fitted) | 22.4 | 0.330 | 0.15 | 1 |
| same, **after re-fitting τ_v 0.89 / d 0.16 with `fit_lag.py`** | 17.9 | 0.199 | 0.29 | 0 |
| PX4 XY P 2.6 (faster loop) | 17.6 | 0.137 | 0.28 | 0 |
| VIO latency 120 ms | 17.6 | 0.111 | 0.25 | 0 |
| odom-vs-PX4 heading residual 5° | 18.4 | 0.171 | 0.20 | 0 |
| VIO noise 3 cm | 17.7 | 0.179 | 0.20 | 0 |
| drag 0.5 /s | 21.9 | 0.138 | 0.32 | 0 |
| VIO drift 2 cm/s random walk (localization's job) | 17.6 | 0.322 | 0.10 | 1 |

Velocity-estimate scale error (PX4's EKF2 velocity = scale × true, so the vehicle flies 1/scale × the
command): scale 0.8 → 6 touches, 0.6 → 15 touches, 1.25 → slow but clean. On the drone EKF2 and our pose
both come from cuVSLAM, so a VIO scale error is consistent in the frame we track in (the world path is
scaled instead: localization's job). It only bites when the pose source differs from what PX4 fuses: sim
ground truth with a broken VIO (today), or a localization scale correction.

**Lesson: fit τ_v / t_delay from logs (`tools/fit_lag.py`, on sim and real bags) before trusting speed.**
Under-estimating τ_v is what produces touches. Over-estimating is benign.

### Speed (course v3), takeoff/landing

| plan MaxVelMag | cap | plan s | course s | max xt | touches |
|---|---|---|---|---|---|
| 1.2 | 1.5 | 17.03 | 17.6 | 0.124 | 0 |
| 1.35 | 1.5 | 16.52 | 17.5 | 0.108 | 0 |
| 1.6 | 1.8* | 15.11 | 16.0 | 0.087 | 0 |
| 2.0 | 2.3* | 14.64 | 15.3 | 0.212 | 0 |

\* needs organizers' OK to raise `max_speed_mps`. On this course the speed limit is worth ~2 s; the
planner's random route alone varies the plan by ±1 s, so the node now keeps the **fastest of 3 validated
plans** (`planner_candidates`).

Takeoff/landing (same course): climb 1.0 m/s (PX4's 3 s takeoff ramp dominates), settle 0.3 s, start the
course 0.15 m below hover height, descend 0.8 m/s then 0.2 m/s below 0.4 m: mission 23.4 → **22.5 s**,
touchdown error 1–2 cm. Final descent at 0.3 m/s below 0.25 m (instead of 0.2 m/s below 0.4 m): mission 21.95 → **21.5 s**.

### Path clock (course v3)

Starting the course clock at full rate (the plan starts at rest) removes the last 0.5 s: course
**17.06 s on a 17.03 s plan**. Running the clock faster than planned (`speed_factor` = `tracker.taudot_max`)
stays clean, with the planner's tilt/thrust margins absorbing the higher accelerations:

| speed_factor | mission s | course s | max xt | min clear | touches |
|---|---|---|---|---|---|
| 1.0 | 21.94 | 17.06 | 0.168 | 0.25 | 0 |
| 1.1 | 20.87 | 15.98 | 0.168 | 0.30 | 0 |
| 1.15 | 20.66 | 15.79 | 0.168 | 0.30 | 0 |
| 1.2 | 20.51 | 15.65 | 0.168 | 0.30 | 0 |

Random seeds 1/4/7 at 1.15: 11.3/7.9/11.8 s courses, 0 touches. Default stays 1.0 until Gazebo and VIO
at speed are checked.

### Random courses (`tools/make_random_map.py`, seeds 1, 3–8; rotated layouts, random launch heading)

Plans at MaxVelMag 1.2. Seeds 1, 3, 4, 8 crashed the planner before the DroneHacks `sfc_gen.hpp` fix (OMPL InformedRRTstar destructor
segfault after a straight 2-state solution; `planner/third_party/gcopter/UPSTREAM.md` change 3, regression test
`planner/tests/run_tests.py` section 5). Seed 2 generates an invalid course.

| seed | mission s | course s | plan s | max xt | min clear | touches | checkpoints | cp miss | land err |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 17.44 | 12.88 | 12.34 | 0.154 | 0.38 | 0 | 4/4 | 0 | 0.024 |
| 3 | 11.52 | 6.46 | 5.90 | 0.162 | 0.40 | 0 | 1/3 | 0.030 | 0.014 |
| 4 | 14.66 | 9.26 | 8.76 | 0.173 | 0.45 | 0 | 4/4 | 0 | 0.009 |
| 5 | 17.62 | 12.66 | 12.11 | 0.170 | 0.36 | 0 | 3/3 | 0 | 0.016 |
| 6 | 14.48 | 9.50 | 8.96 | 0.162 | 0.35 | 0 | 3/3 | 0 | 0.014 |
| 7 | 18.70 | 13.48 | 12.92 | 0.160 | 0.35 | 0 | 1/4 | 0.023 | 0.007 |
| 8 | 14.98 | 10.34 | 9.81 | 0.158 | 0.35 | 0 | 3/3 | 0 | 0.005 |

Misses on 3 and 7 are 2–3 cm outside boxes that the plan itself only grazes (closest approach at
86–93 % of the half-width); they are inside the openings. Max xt includes the climb-out of
`course_start_dz` (0.15 m).

### ROS 2 end-to-end (`tools/e2e_docker.sh`, Humble arm64, `kinsim_ros.py` stand-in)

Real `mission_tracker` node + planner at startup: idle → takeoff → course → landing → landed → done,
course 18.3 s on a 17.5 s plan, touchdown 4 cm from the goal. A "go" before the plan was ready was
correctly refused (the node publishes nothing until it has a plan).

### Return leg (optional, `return_config`; needs the organizers' OK)

Touch-and-go: after PX4 reports landed on the pad, wait 0.5 s (PX4 auto-disarms only after
`COM_DISARM_LAND` = 2 s), climb again in the same Offboard session, fly a second plan, land. With
`return_config:=reverse` (generic: pad → over the launch mark, no checkpoints; a real return over the
barrier needs its own config): return plan 4.7 s, return 9.5 s incl. takeoff ramp and landing, final
touchdown 4 mm from target, total 31.5 s. ROS e2e: idle → takeoff → course → landing → landed → return →
landing → landed → done.

## Gazebo slot 0 (`path_following/sim/slot0.sh`)

**Status 16:15 PST: cameras fixed (5e8f6f8); VIO tracks for 26 s, then breaks at the C2a window.** The cameras
had rendered saturated white (the course OBJ had no vertex normals; fixed by the coordinator). Mono images now
have mean 43 and std 13. Results before 5e8f6f8 (gt02–gt06) flew on a frozen VIO and are invalid as tracking
results.

VIO vs ground truth (`tools/vio_err.py`, VIO aligned at takeoff = what a tracker with a perfect launch pose sees):

| run | mode | speed | VIO error to 26 s | at the C2a window (≈26–28 s) | outcome |
|---|---|---|---|---|---|
| gt10 | gt | 1.0, default lag | < 0.05 m, scale 0.99 | +0.29 m jump in x, then steady | done, 5/5, 0 touches, course 22.0 s (plan 17.4), max xt 0.18, min clear 0.25 |
| gt11 | gt | 1.0, sim lag | < 0.05 m | diverges to 28 m (VIO path 68 m vs 18 m) | PX4 flew on it: 1 touch, max xt 1.03 |
| vio10 | vio | 1.0, sim lag | < 0.05 m | jump (−7.8, 0, −3.9) m, then a constant offset | tracker held, 8 m "off path", until timeout |

- gt mode still depends on cuVSLAM, because PX4's EKF2 fuses it. When VIO breaks, gt runs break too.
- The dark tunnel (checkpoint 4) is passed with VIO error < 5 cm. The failure is at the window in the turned
  wall. Likely the wall fills the stereo view on approach (texture, or the near clip), then the view jumps
  once through the window. Reported; worth checking on the real drone's OAK view too.
- PX4 velocity loop through the interface (gt10, `tools/fit_lag.py`): tau_v 0.66 s, dead time 0.24 s
  (first order), second order overdamped (ζ ≈ 4.5) with dead time 0.12 s, steady-state gain 1.06.
  Kinsim with a 0.14 s link delay reproduces that fit (tau 0.63, d 0.24). On that plant, tracker lag
  assumptions (course v3, plan 17.03 s):

  | tracker tau / d | speed 1.0: course s, max xt, min clear, cp | plan ×0.85 (faster) |
  |---|---|---|
  | 0.55 / 0.10 (default) | 18.36, 0.153, 0.20, 4/5 | 16.86, 0.161, 0.25, 5/5 |
  | 0.66 / 0.24 (fit) | 17.06, 0.180, 0.30, 5/5 | 15.80, 0.218, 0.27, 4/5 |
  | **0.66 / 0.18** | 17.06, 0.162, 0.30, 5/5 | **15.60, 0.190, 0.335, 5/5** |
  | 0.60 / 0.30 | 17.24, 0.183, 0.30, 5/5 | 16.38, 0.223, 0.25, 4/5 |
  | 0.75 / 0.24 | 17.12, 0.212, 0.29, 4/5 | 17.46, 0.264, 0.25, 4/5 |

  `config/tracker_sim_x500.yaml` is now 0.66 / 0.18. Under-estimating the lag (the default) costs about 1.3 s,
  through the path clock slowing on cross-track error. That's the slow 22.0 s course in gt10.
- Checkpoint scoring now skips a missed box instead of stopping the count (`course.score_checkpoints`).
  A 2 cm miss of box 1 used to score 0/5.

Earlier findings (still valid):

- The chain works end to end: plan at startup (best of 3, ~10 s in the container), zero twists until the
  pilot's go, Offboard, takeoff, course start (gt02–gt05).
- The sim drone is an x500: rotors reach 0.39 m from centre. The plan keeps 0.30 m (DilateRadius for our
  drone) and the 0.93 m maze gaps leave a centred x500 ~7 cm per side, so it clips walls (gt02, gt05 at
  M1; gt04 near the west net). `DilateRadius` > 0.3 needs `VoxelWidth 0.05` (dilation is in whole voxels);
  0.35 plans, 0.4 does not (the launch mark is within 0.4 m of a wall). Asked for a Sky Beta-sized
  collision footprint.
- Takeoff: Offboard → course start 8.5 s with the takeoff climb gain 1.2, **5.0 s with `takeoff_kz` 2.5**.
- After a wall strike EKF2 reported −15 m/s while stationary, and the latency compensation moved the
  node's estimate by 1 m. Fixed: EKF2 velocity is used only while it agrees with the pose velocity, and
  the prediction is capped (`vel_agree_mps`, 0.15 s, max_speed + 0.5).
- Flying above the 1.73 m walls (sysid01/02 at 2.4 m) breaks PX4's height: the rangefinder
  (`EKF2_HGT_REF=2`) sees the wall tops. Relevant for any return over the barrier on the drone.
- rclpy raised `RuntimeError: Unable to convert call argument` inside the executor once (sysid node,
  px4_msgs subscription). mission_tracker now logs executor errors and keeps spinning.

| sim | run | result | mission s | course s | plan s | max xt | rms xt | min clear | touches | checkpoints | cp miss | land err | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
