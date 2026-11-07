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
touchdown error 1–2 cm.

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

Plans at MaxVelMag 1.2. Seeds 1, 3, 4, 8 crash the current planner (OMPL InformedRRTstar destructor
segfault after a straight 2-state solution); their plans here come from a scratch copy with a
straight-line pre-check (reported; `planner/` unchanged). Seed 2 generates an invalid course.

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

## Gazebo slot 0 (`path_following/sim/slot0.sh`)

| sim | run | result | mission s | course s | plan s | max xt | rms xt | min clear | touches | checkpoints | cp miss | land err | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
