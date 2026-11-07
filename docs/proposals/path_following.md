# Proposal: path streaming + path following

Status: **for review**. Nothing here is built or flown yet.
Written Sat 7 Nov 2026, 02:30 PST. Scored runs start Sat 22:00 PST; code due Sun 16:00 PST.

Read with `docs/CONTRACTS.md`, `docs/sessions/path_following.md` and `planner/README.md`.

## TL;DR

- One ROS 2 Humble Python package, `team_path_following`, with one node, `mission_tracker`.
  It runs the GCOPTER planner at startup, converts its output to `map` ENU once, and at 50 Hz
  publishes a **velocity** setpoint (path velocity looked ahead by the PX4 lag, plus feedback on
  position error) on `/skybeta/setpoint/twist`. It never publishes a pose.
- The path clock slows down when tracking error grows (time-scaling), so the reference never runs
  away from the drone and corners are not cut.
- Libraries: numpy/scipy only (both already in the drone image). No acados, TOPP-RA or MPCC for
  Saturday night. Each is either blocked on ARM packaging, needs a rebuilt team image, or targets
  a control interface we do not have. A preview MPC (closed form, numpy) is the Sunday upgrade.
- Biggest finding from the interface source: **if we ever send a pose and then the twist stream
  stops, the drone flies back to that pose at up to 12 m/s.** Hence the rule above: twists only.

## 1. What actually reaches PX4

Read from `~/skybeta/src/skybeta_flight_interface/` on `skybeta-12` (skybeta 0.3.2, branch
`fleet-ready`, commit `baf21fd`), read-only. The drone's `~/skybeta/docs/` is deleted in the
working tree. I read the guide from git (`git show HEAD:docs/competitor_guide.md`).

**Path.** `/skybeta/setpoint/twist` (`TwistStamped`) → `twist_setpoint_cb` → clamp → ENU→NED →
stored as the target. A timer at `setpoint_rate_hz` publishes `OffboardControlMode` then
`TrajectorySetpoint` on `/fmu/in/*`. Our message rate and PX4's input rate are decoupled: PX4 gets
the latest twist, zero-order-held, at `setpoint_rate_hz` (10 Hz by default, so up to 100 ms extra
delay).

**Fields in velocity mode.** `velocity = enu_to_ned(v)` = `(v_y, v_x, -v_z)`, `yawspeed =
-angular.z` (NaN if not finite). `position`, `acceleration` and `yaw` are NaN. Flags: `velocity=True`
only. So PX4 runs only its velocity PID: no position loop, no acceleration feedforward, no
smoothing. **Height is velocity-controlled too, so z drifts unless we close the loop on it.**
`header.frame_id` is ignored. The twist must be in the `odom` ENU *world* frame, not the body frame.

**Clamps.**
- Velocity: if `|v|` (3D norm, including z) > `max_speed_mps`, the whole vector is scaled down,
  so direction is kept. Reported as `clamped: ["max_speed"]` in `/skybeta/state`.
- **Twists get no altitude clamp.** `min_altitude_m`/`max_altitude_m` apply only to poses.
  We must enforce the 68 in (1.727 m) ceiling ourselves. The upside is that a twist can descend
  all the way to the pad (a pose is floored at 0.3 m).
- Yaw rate is not clamped.
- NaN/inf twists are dropped and the previous target stays.
- Velocity is **not** rotated by `odom_to_ned_yaw_offset_rad` or anchored. Only poses are.
  Velocity commands do not care about the odom↔EKF2 translation offset, which is another reason
  to prefer them.

**Timeouts and fallbacks.**
- A twist older than `setpoint_timeout_s` (1.0 s, not a launch argument) drops the interface to
  `hold`. Hold flies to `hold_position_ned`, which is **the last pose setpoint ever received**, or,
  if none was ever sent, PX4's current local position re-read every tick (≈ stop where you are).
  `/skybeta/hold` does the same. **So we never send a pose**, not even on the ground before arming.
  The competitor example does send one; don't copy that part.
- If the interface's own stream stalls for `COM_OF_LOSS_T` = 1 s, PX4 leaves Offboard
  (`COM_OBL_RC_ACT` = 0, Position). Offboard never comes back until disarm. The run is over.

**Getting into Offboard** (`auto_offboard.py`). The pilot arms on Altitude, then switches to
Position while the drone is still landed. The interface then requests Offboard once, but only if
PX4 has a valid local position, there is no hold, and **a team setpoint arrived in the last second**.
So we stream zero twists from boot. PX4 disarms 10 s after arming if the drone hasn't taken off
(`COM_DISARM_PRFLT`), so the takeoff climb must start as soon as `offboard_active` goes true.

**Launch parameters** (`competitor.launch.py`): `max_altitude_m` (3.0), `min_altitude_m` (0.3),
`max_speed_mps` (1.5), `setpoint_rate_hz` (10), `allow_mode_request` (false; sim only),
`auto_offboard_on_arm` (must be true), `odometry_topic`. The guide says to tighten the envelope from
the launch file and **not loosen it without asking**, so raising `max_speed_mps` is an organizer
question. `setpoint_rate_hz` is not part of the envelope, but we'll confirm that too (§7).
Our launch file must start `flight_interface` exactly as `competitor.launch.py` does, and we set
`COMPETITOR_PKG`/`COMPETITOR_LAUNCH` in `/etc/skybeta/drone.env` (allowed).

**PX4 velocity loop** (fleet parameter files in `~/skybeta/px4/params/`; not yet confirmed on
drone 12's FC itself):

| Param | Value | Meaning for us |
|---|---|---|
| `MPC_XY_VEL_P_ACC` / `I` / `D` | 1.8 / 0.4 / 0.2 | horizontal velocity loop ≈ first-order lag, τ ≈ 1/1.8 ≈ 0.55 s |
| `MPC_Z_VEL_P_ACC` / `I` | 4.0 / 2.0 | vertical τ ≈ 0.25 s |
| `MPC_XY_P` | 0.95 | what PX4's own position loop would use; our starting `Kp` |
| `MPC_TILTMAX_AIR` | 45° | acceleration limit ≈ g·tan 45° ≈ 9.8 m/s² (not a constraint at 1.5 m/s) |
| `MPC_XY_VEL_MAX` | 12 m/s | how fast a far pose would fly |
| `MPC_TKO_RAMP_T` | 3 s | takeoff ramp, also applies in Offboard |
| `COM_DISARM_LAND` | 2 s | auto-disarm after landing |
| `EKF2_EV_CTRL` / `EKF2_EV_DELAY` | 9 (h-pos + yaw) / 60–70 ms | EKF2 heading follows VIO, so `odom` and PX4-local axes agree up to a small yaw residual |
| `EKF2_HGT_REF` | 2 (rangefinder) | PX4 height ≠ VIO z; we use only VIO/`odom_map` z |

Other drone facts: Jetson load average ≈ 4.3 on 6 cores with only the stack running, 3.7 GB RAM
free. The container has Python 3.10, numpy 1.26, scipy 1.15, Eigen 3.4 and OMPL 1.7 (so the planner
builds in the stock image). casadi, acados, toppra, osqp and cvxpy are all missing.

## 2. Approaches compared

The plant we control is not a quadrotor. It is PX4's velocity loop: a ~0.55 s first-order lag,
plus ~50–150 ms of delay (VIO latency + 10 Hz zero-order hold + DDS), with a 1.5 m/s norm cap.
Anything that outputs attitude, rates or thrust cannot be used.

| Approach | Gain over plain waypoints | Fits our interface | ARM / Humble cost | Verdict |
|---|---|---|---|---|
| **A. Feedforward + P(D) + lag compensation + time-scaling** | Large: no 1.6 m lag, no corner cutting | Native | numpy only | **Build first, fly Saturday** |
| **B. Preview / unconstrained linear MPC** on a first-order-lag model, solved in closed form | Better corners: anticipates turns over a 1–1.5 s horizon, handles delay explicitly | Native (output = v_cmd) | numpy/scipy, gains precomputed at startup | **Sunday upgrade if A is stable** |
| C. acados constrained MPC (HPIPM) with corridor half-planes | Hard speed/corridor constraints | Native if we write the model; reference repos (`px4-mpc`, `quadrotor-ros2`) output wrench/rates to `/fmu/in`, so only their structure is reusable | Needs a team Docker image: acados C build + `t_renderer` arm64 binary (exists since tera_renderer 0.2.0) + `acados_template`. The planner doesn't export its corridor polytopes today | Not before scored runs. Revisit only if B's saturation is the bottleneck |
| D. MPCC progress maximization (Romero et al.; `dq_mpcc` runs on Orin NX at 100 Hz) | Picks speed online from tracking quality | Published code outputs rates/thrust, not velocity | acados + casadi, team image | Not used. A's time-scaling law is the cheap version of the same idea |
| E. TOPP-RA re-timing | Quick speed sweeps without re-planning | Offline | PyPI `toppra` has no aarch64 wheel and no sdist | Not needed: global time-scaling of a polynomial path is exact (v/k, a/k²), and re-running GCOPTER with a new `MaxVelMag` takes seconds |

At a 1.5 m/s cap the tracker is unlikely to be the limit. VIO is: motion blur, the dark tunnel,
yaw rate. So the plan puts effort into A being robust (watchdogs, time-scaling, clearance
monitoring) rather than into a solver.

## 3. Architecture

```
planner/ (gcopter_planner binary, run once at node start)
   └─ out/path.bin  (PX4-local NED rel. launch, 100 Hz rows)
            │ load + convert once (pathfile.py)
            ▼
   Trajectory in map ENU ───────────────► /team/plan/path (nav_msgs/Path, transient-local, 10 Hz decimated)
            │
/team/odom_map (or fallback) ──► mission_tracker @ 50 Hz ──► /skybeta/setpoint/twist (odom ENU)
TF map→odom ───────────────────►   state machine + tracker ──► /team/mission/state
/team/loc/status ──────────────►                            ──► /team/tracker/debug
/skybeta/state (JSON) ─────────►
/fmu/out/vehicle_local_position (read only: low-latency velocity, landed)
```

### Package `team_path_following` (ament_python)

Pure-Python modules with no ROS imports, all unit-tested on the laptop:

| Module | Contents |
|---|---|
| `pathfile.py` | read `DHPATH` v1 → numpy; NED→map conversion; `Trajectory.sample(τ)` (linear interpolation of p, v, a, yaw at 100 Hz rows); nearest-point/arc-length helpers |
| `frames.py` | SE(2)+z transforms: map↔odom; yaw wrap |
| `tracker.py` | the control law below; `TrackerParams` dataclass |
| `timelaw.py` | path-clock rate from errors and loc status |
| `mission.py` | state machine (§3.4), a pure function of inputs → (state, command) |
| `clearance.py` | voxel distance field from the DHMAP map (`scipy.ndimage.distance_transform_edt`) for min-clearance monitoring and slow-down |

ROS glue: `node.py` (`mission_tracker`). `launch/mission.launch.py` = `flight_interface` exactly as
in `competitor.launch.py` + `mission_tracker`. `config/tracker.yaml` holds the gains, and the course
config path is a parameter. No course numbers in code.

Tools: `tools/fake_px4.py`, a kinematic stand-in for interface + PX4 (zero-order hold at
`setpoint_rate_hz`, norm clamp, first-order lag with fitted τ and dead time, VIO latency/noise).
It runs the tracker on the course and on random maps in seconds on a laptop.
`tools/make_random_map.py` generates random wall maps, since `make_wall_map.py` is a fixed course.
`tools/eval_run.py` reads a bag and writes metrics.

### 3.1 Frames

All tracking math is done in `map`. The path is converted from NED once at load; the final command
is rotated into `odom` at publish time.

**Planner NED → map ENU** (inverse of `planner/README.md` "Frames"; the 2×2 matrix is symmetric and
its own inverse). `ψ0 = StartHeadingDeg`, `S = StartPos`:

```
p_map.x = S.x + cos ψ0 · x_ned + sin ψ0 · y_ned
p_map.y = S.y + sin ψ0 · x_ned − cos ψ0 · y_ned
p_map.z = S.z − z_ned
v_map, a_map: same rotation, no offset (z negated)
θ_map = ψ0 − yaw_ned            θ̇_map = −yaw_rate_ned
```

Unit test: the README's worked example (`S=(2,3,0)`, `ψ0=90°`) and a round-trip against the
planner's own forward formula.

**map → odom.** `T_map_odom = (t, ψ)` from TF (localization). Then:

```
p_odom = Rz(−ψ) (p_map − t)          v_odom = Rz(−ψ) v_map
θ_odom = θ_map − ψ                   θ̇_odom = θ̇_map
```

Fallback until `/team/odom_map` exists (contract): odometry from `/visual_slam/tracking/odometry`,
and `T_map_odom` built **at the go signal** from the current odom pose and the launch pose:
`T_map_odom = T_map_launch · T_odom_base(now)⁻¹`. That is robust to the stack having started
somewhere other than the mark. It assumes the drone sits on the mark at `StartHeadingDeg` when
armed. The z of `base_link` on the pad (landing-gear height) goes into `T_map_launch`;
agree on this with localization.

**Command into `/skybeta/setpoint/twist`:** `linear = v_odom`, `angular.z = θ̇_odom command`, stamp = now.

**Residual yaw between `odom` and PX4-local.** EKF2 fuses VIO yaw, so the axes should agree. A
residual rotates our velocity commands, and feedback absorbs a few degrees. Measure it in sim and on
the bench (odom yaw vs `/fmu/out/vehicle_attitude` yaw). We can't change the interface, so if it is
above ~3° we pre-rotate inside our node, as a parameter.

### 3.2 The tracker (approach A)

Every 20 ms, with path clock τ:

1. **State estimate.** Position from the latest odometry, forward-predicted by its age:
   `p ≈ p_msg + v (now − stamp)`. Velocity from `/fmu/out/vehicle_local_position` (EKF2, IMU rate,
   low latency), else from odometry twist rotated to world. Odometry age > 0.25 s → zero twist
   (watchdog).
2. **Reference with lag compensation.** `p_r = p(τ)`, `v_ff = v(τ + T_lead)`, `θ_r = θ(τ)`.
   `T_lead` ≈ τ_v + dead time (≈ 0.6–0.7 s to start; fitted in sim T1). This is the velocity
   channel's version of acceleration feedforward: for a first-order velocity loop,
   `v_cmd = v_r + τ_v a_r` tracks exactly, and the look-ahead form keeps |v_ff| within the path's
   own speed limit.
3. **Feedback.** `e = p_r − p`, split into along-track `e_a = (e·t̂) t̂` and cross-track
   `e_c = e − e_a` using the path tangent t̂. `v_fb = Kc e_c + Ka e_a + Kz e_z` (start: Kc = 1.2,
   Ka = 0.6, Kz = 1.5 s⁻¹), plus an optional small integral term with anti-windup for drag and
   trim offsets.
4. **Saturation with priority.** The budget is `v_max = max_speed_mps − margin` (0.1). The
   cross-track and vertical corrections get the budget first, then whatever along-track speed fits.
   This avoids the interface's uniform scaling, which would shrink corrections in a corner.
   If anything was trimmed, the clock slows (step 6).
5. **Altitude ceiling** (twists aren't clamped): if `z_map > z_ceiling − 0.1`,
   `v_z ≤ −Kz (z_map − z_ceiling + 0.1)`. `z_ceiling` comes from config (1.6 m by default, below
   1.727 m).
6. **Time law (time-scaling).**
   `τ̇ = clip(1 − max(0, |e_c| − e0)/e_span − max(0, −e·t̂ − e0)/e_span, 0, 1) · s_loc · s_clear`,
   with `e0 = 0.10 m`, `e_span = 0.30 m`. The clock stops if the drone is ≥ 0.4 m off or behind.
   `s_loc` = 1 on OK, 0.5 on WARN, 0 on ERROR (contract: slow on WARN, hold on ERROR).
   `s_clear` drops to 0.5 when map clearance at the drone is < `DilateRadius`.
   τ̇ is rate-limited (±1 s⁻²) so speed changes are smooth.
7. **Yaw.** `θ̇_cmd = θ̇_r + Kψ wrap(θ_r − θ)` (Kψ = 1.5), clipped to `YawRateMaxDeg`.
   Option `yaw_mode: fixed` holds the launch heading, in case yawing costs VIO tracking.

Speed sweeps: the planner's `MaxVelMag` (re-plan) or a global time-scale `k` on the loaded path.
Plan with `MaxVelMag ≈ max_speed_mps − 0.25` so feedback has headroom under the cap.

### 3.3 Preview MPC (approach B, Sunday)

Per axis: `p⁺ = p + Δt v`, `v⁺ = v + (Δt/τ_v)(u_{k−d} − v)`, with a d-step delay from the fitted
dead time. Δt = 0.1 s (one interface tick), N = 15. Cost: Σ ‖p − p_r‖²_Q + ‖v − v_r‖²_R + ‖Δu‖²_S over
the future reference from the trajectory. With no constraints this is a linear least-squares problem,
so the gain matrices are precomputed once with numpy and each tick is a matrix-vector product.
Constraints are handled as in A (priority saturation + time-scaling). Same node, `controller: preview`.
It only ships if it beats A on the sim sweep.

### 3.4 Mission state machine (`/team/mission/state`)

| State | Enters when | Output |
|---|---|---|
| `idle` | boot; planner OK, path loaded | zero twist at 50 Hz (keeps auto-offboard eligible). **Never a pose.** |
| `takeoff` | `/skybeta/state.offboard_active` true | xy feedback on the launch point; `v_z` ramps up to the `HoverHeight` target (≤ 0.6 m/s). Must leave the ground within the 10 s pre-flight disarm window |
| `course` | settled: \|e\| < 0.1 m, \|v\| < 0.15 m/s for 1 s (max 3 s) | tracker, τ from 0 |
| `landing` | τ = T_end and at the goal within 0.15 m | xy hold on the goal (pad); `v_z` = −0.4 m/s, then −0.2 below 0.3 m |
| `landed` | PX4 `landed` (`vehicle_land_detected` / `/skybeta/state.landed`) | `v_z` = −0.2 for 3 s so PX4 auto-disarms (`COM_DISARM_LAND`) |
| `done` | disarmed | zero twist |
| `return` | only if organizers allow a re-arm (§7) | second path (goal → start), same tracker |
| `abort` | `offboard_active` true→false, planner failure, or unrecoverable error | zero twist, mission frozen. Never tries to re-enter Offboard |

Planner failure (exit 1) is retried 3 times with the RRT seed varying by run. If all fail, we stay
in `idle` and log it. Arming then gives "no autonomy", which is the safe outcome.

## 4. Topics and parameters

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/team/odom_map` | `nav_msgs/Odometry` | in | primary pose (map). Fallback `/visual_slam/tracking/odometry` |
| TF `map → odom` | tf2 | in | latest correction; fallback static, built at go |
| `/team/loc/status` | `diagnostic_msgs/DiagnosticStatus` | in | OK/WARN/ERROR → `s_loc` |
| `/skybeta/state` | `std_msgs/String` JSON | in | `offboard_active`, `armed`, `landed`, `clamped` |
| `/fmu/out/vehicle_local_position` | `px4_msgs` | in (read only) | velocity, `xy_valid`. Name resolved like the interface does (`_v<N>`) |
| `/skybeta/setpoint/twist` | `TwistStamped` | **out** | the only setpoint we publish |
| `/team/plan/path` | `nav_msgs/Path` | out | transient-local |
| `/team/mission/state` | `std_msgs/String` | out | contract values |
| `/team/tracker/debug` | `std_msgs/Float64MultiArray` (layout documented) | out | τ, τ̇, e_a, e_c, e_z, v_ff, v_fb, v_cmd, clearance, trimmed flag |

Launch arguments we set on `flight_interface`: `setpoint_rate_hz` (sim sweep 10/20/30/50),
`max_speed_mps` (1.5 unless organizers allow more), `max_altitude_m` 1.7 (tighter, allowed),
`auto_offboard_on_arm` true, `allow_mode_request` true **in sim only**.

## 5. Test plan, sim slot 0

Each test records a bag (setpoints, `/fmu/in/trajectory_setpoint`, `/skybeta/state`, odometry, ground
truth, `/team/tracker/debug`). `tools/eval_run.py` writes the results row to
`docs/results/path_following.md`: lap time, max/RMS cross-track error (ground truth vs path), min
clearance (ground truth vs map distance field), checkpoints entered in order, `clamped` count,
time in Offboard.

| # | Test | Pass |
|---|---|---|
| T0 | `fake_px4` on laptop: course + 5 random maps, all modules' unit tests | finishes; max e_c < 0.15 m at 1.2 m/s |
| T1 | **Interface characterization**: 1 m/s velocity steps per axis and 1.5 m/s diagonal at `setpoint_rate_hz` 10 and 50; fit τ_v and dead time; check the norm clamp and `clamped` | numbers in the results file; `fake_px4` updated with them |
| T2 | **Start sequence**: zero-twist idle → `pilot.py go` → Offboard → twist takeoff → settle; also arm with no team node (must stay out of Offboard) | Offboard within 2 s of go; airborne < 5 s after arming |
| T3 | **Fallback safety**: kill the tracker mid-course; confirm the drone stops in place (no pose was ever sent). Stall odometry → watchdog zero twist | stops within 0.5 m, no flyback |
| T4 | Full course, ground-truth odometry as tracker input, 1.0 m/s plan | all checkpoints in order, min clearance > 0.15 m |
| T5 | Same with cuVSLAM in the loop (realistic) | as T4; records VIO-vs-GT error along the course |
| T6 | **Sweep**: plan 0.8 → 1.25 m/s (cap 1.5) × `setpoint_rate_hz` 10/30/50 × `T_lead` 0/0.4/0.7 s; above the cap only if organizers allow | pick the fastest config with zero touches over 3 runs each |
| T7 | Loc status WARN/ERROR injection (fake publisher); `/team/odom_map` correction jumps of 0.2 m | slows / holds; no lurch > 0.3 m/s from a jump |
| T8 | Landing in the ring, auto-disarm | lands within 0.2 m of the pad centre; disarms |
| T9 | Preview MPC vs A on T6's best config | ships only if better on all of time, max e_c, clearance |

Known sim gaps (`docs/SIM.md`): x500 dynamics, ideal cameras. So gains get a final check
low and tethered on the real drone, props off first.

## 6. Milestones (≈ 20 h to scored runs)

| By (PST) | Milestone | Gate |
|---|---|---|
| Sat 03:30 | this proposal reviewed | — |
| Sat 07:00 | M1: package skeleton, `pathfile`/`frames` + tests, `/team/plan/path`, `fake_px4`, planner run on the course config | T0 frame tests |
| Sat 10:00 | M2: tracker A + time law + mission state machine in `fake_px4` (course + random maps) | T0 |
| Sat 13:00 | M3: slot 0: T1, T2, T3, T4 (ground truth) | full course in sim |
| Sat 16:00 | M4: T5 (cuVSLAM), T7, T8; first sweep T6 | results table |
| Sat 19:00 | M5: on the drone: planner builds in the stock image, node in `~/team_ws`, `COMPETITOR_PKG` set, props-off dry run (idle → arm → Offboard → zero-twist takeoff command visible on `/fmu/in`), then low tethered hover with the pilot | pilot sign-off |
| **Sat 22:00** | **scored runs with tracker A** at the speed that passed T6 with margin | — |
| Sun 08:00–11:00 | M6: retune from Saturday's logs; preview MPC (T9) if A is stable; return leg if allowed | no regression in sim |
| Sun 12:30 | repo link; README credits (GCOPTER, numpy, scipy) | — |
| Sun 16:00 | all flown code pushed | — |

PR when M3 works in sim (per the session doc).

## 7. Open questions

**For the organizers**
1. **Return bonus (rule 7.10):** it needs a second takeoff after disarming. The interface can't arm,
   and Offboard comes only from arm → switch to Position on the ground. Can the safety pilot re-arm
   and switch on the pad for the return, and does that still count?
2. **`max_speed_mps` above 1.5:** the guide says not to loosen the envelope without asking. May we,
   and up to what?
3. **`setpoint_rate_hz` 30–50 Hz:** OK? Any TELEM2/uXRCE bandwidth concern?
4. **Checkpoints:** does `course_v3.yaml`'s list (M1, M2, M3, tube, C2a window) match the official
   checkpoint lines?
5. **Launch:** do we place the drone on the launch mark at a known heading, and may we restart the
   stack there (restart re-zeroes the VIO origin) before each run?
6. **Landing:** does "on the pad" need a disarm, and is PX4's auto-disarm in Offboard what you expect?
7. Reading `/fmu/out/*` (not publishing) from team code: we assume it's fine. Confirm.
8. The drone's `~/skybeta/docs/` files show as deleted in its working tree. Intended?

**For the team**
- Planner: could it also write its corridor polytopes (needed only for an acados corridor
  constraint)? Who owns the real vehicle mass/thrust for `course_v3.yaml`?
- Localization: `base_link` height above the pad in `T_map_launch`, the `/team/odom_map` latency
  budget, and how big a `map → odom` jump can be (feeds T7).
- PX4 velocity-loop gains on drone 12's FC: confirm with a QGC parameter dump (read only).
  The values above come from the fleet files.

## References

- Flight interface source and competitor guide on `skybeta-12` (skybeta 0.3.2).
- GCOPTER: https://github.com/ZJU-FAST-Lab/GCOPTER (vendored in `planner/`).
- acados https://github.com/acados/acados; tera_renderer arm64 builds since v0.2.0
  https://github.com/acados/tera_renderer/releases
- px4-mpc https://github.com/Jaeyoung-Lim/px4-mpc (acados + ROS 2, wrench/rate modes on `/fmu/in`)
- quadrotor-ros2 https://github.com/clementdumeril/quadrotor-ros2 (acados NMPC, ROS 2 Jazzy)
- Romero et al., MPCC for time-optimal quadrotor flight https://arxiv.org/abs/2108.13205;
  dq_mpcc (ROS 2, acados, Orin NX) https://github.com/LASER-Robotics/dq_mpcc;
  CMPCC https://github.com/ZJU-FAST-Lab/CMPCC
- TOPP-RA https://github.com/hungpham2511/toppra (PyPI: x86 wheels only, no sdist)
