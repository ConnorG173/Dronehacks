# Session: path streaming + path following

Read `docs/CONTRACTS.md` first. You own everything from "planned path" to "setpoints on `/skybeta/setpoint/*`".

## Goal

At mission start, plan once from the map, then stream setpoints that keep the drone on the path as fast as
tracking allows. Score is checkpoints first, then time. A finish beats any speed.

## What already exists: `planner/`

A teammate's GCOPTER planner (ZJU FAST Lab, MIT; vendored with two bug fixes and parity tests). It reads
`planner/config/course_v3.yaml` + `planner/maps/course_v3.bin` and writes `out/path.bin`: rows of
`t, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate` at 10 ms, in PX4 local NED relative to the launch pose.
Read `planner/README.md`. **Don't rewrite the planner.** Improve its inputs (real vehicle mass/thrust/drag,
limits you trust) and make it run on the Jetson at mission start (C++, Eigen + OMPL, builds on ARM).

## Why the tracker is where the edge is

On the drone, the organizers' flight interface is the only path to PX4. It sends PX4 position **or**
velocity at 10 Hz, never both, no acceleration, and PX4 Offboard does no smoothing. So the planner's
pos/vel/acc rows cannot be passed through as they are:
- Position rows only → PX4 lags by about speed / `MPC_XY_P` (≈1.6 m at 1.5 m/s with default gain) and cuts corners.
- Far position waypoints → PX4 lunges at up to 12 m/s and overshoots.

Do this instead: look up the path row for "now", and publish a **velocity** setpoint =
path velocity (feedforward) + gains × position error, from odometry at full rate. Raise the interface's
`setpoint_rate_hz` (30–50) and `max_speed_mps` only as far as tracking and VIO hold up.
Two frame conversions on the way: planner NED → `course` ENU, then `course` → cuVSLAM `odom` ENU with the latest
localization correction (contracts).

Note: the organizers' cloud lab code (`refnav/`) publishes `/fmu/in/trajectory_setpoint` directly with full
pos/vel/acc. That works in the lab, not on the drone, where it would fight the flight interface.

## Proven open-source pieces to use

1. **Baseline tracker:** velocity feedforward + PD on position error, saturated, with time-scaling:
   if cross-track error grows, slow the path clock down instead of letting the drone fall behind.
2. **MPC tracker to push speed:** [acados](https://github.com/acados/acados) on a simple model (point mass, or
   PX4's velocity loop as a first-order lag fitted from sim and real logs), output = velocity setpoint,
   constraints = speed, acceleration, and the planner's corridor polytopes as half-planes. References:
   [Jaeyoung-Lim/px4-mpc](https://github.com/Jaeyoung-Lim/px4-mpc) (acados + PX4 + ROS 2),
   [clementdumeril/quadrotor-ros2](https://github.com/clementdumeril/quadrotor-ros2) (acados NMPC, ROS 2).
3. **Idea to try:** MPCC-style progress maximization ([Romero et al.](https://arxiv.org/abs/2108.13205)) along the
   planner's path, so speed is chosen online from tracking quality instead of a fixed time law.
4. **Time law alternative:** re-time the planner's geometric path with [TOPP-RA](https://github.com/hungpham2511/toppra)
   at a chosen speed limit, for quick speed sweeps without re-planning.

## Mission logic you own

`/team/mission/state`: takeoff → course → landing → landed → (return). Take off to `HoverHeight`, settle, stream,
descend onto the pad (inside the outer ring), let PX4 disarm. Slow down on `/team/loc/status` WARN; hold on ERROR.
Check `clamped` in `/skybeta/state`: a silently clamped setpoint looks like a tracker bug.

**Open questions for the organizers:** (1) the return bonus (rule 7.10) needs a second takeoff after disarming,
but the flight interface cannot arm; (2) the planner's checkpoint list is read off the CAD; confirm it
matches the official checkpoint lines.

## Milestones

1. ROS 2 node that loads `path.bin`, converts frames, and publishes `/team/plan/path` for viewing.
2. Baseline tracker flies the full course in our Gazebo sim with ground-truth odometry, through the flight interface.
3. Same with cuVSLAM odometry in the loop (the realistic case).
4. Speed sweep: planner `MaxVelMag` / re-timed path 1.0 → 3.0 m/s. Record time, max cross-track error,
   min wall clearance, touches.
5. MPC tracker; beat the baseline on the same sweep.
6. Planner builds and runs on the Jetson at mission start; real vehicle parameters in the config.
7. Landing in the ring; return (if allowed).

## Test in sim, measure everything

Every change: the course scenario plus at least 5 random maps (`planner/tools/make_wall_map.py` style).
Log a rosbag. Report lap time, max/RMS cross-track error, min clearance, and checkpoints crossed in order.
Keep a results table in `docs/results/path_following.md`.

## Default prompt for this session

> You are the path streaming + path following workstream for our Stanford DroneHacks 2026 team.
> Read `docs/CONTRACTS.md`, `docs/sessions/path_following.md`, `planner/README.md` and `docs/SIM.md`.
> Build a ROS 2 Humble node that runs the existing GCOPTER planner's output (`planner/`, don't rewrite it),
> converts it to the `course`/`odom` frames, and tracks it by publishing velocity setpoints with feedforward to
> `/skybeta/setpoint/twist` through the organizers' flight interface. Start with the baseline tracker, then
> an acados MPC. Use the proven open-source libraries in the doc rather than writing solvers from scratch.
> No course-specific numbers in code. Test every change in the Gazebo sim on the GCP VM (your slot is 0) on the course map and
> on random maps, and record metrics in `docs/results/path_following.md`. Do not modify `~/skybeta` or the
> flight interface. Work on branch `path-following` off `sim-and-plans`, commit small, and open a PR when a
> milestone works in sim.

## Drone-ready status (2026-11-07, 18:30 PST)

Branch `path-following`, local only (not pushed). Quick guide: `path_following/GUIDE.md`; procedure: `docs/DRONE_RUN.md`.

**Deploy: `drone-conservative-v1`** (git tag; `launch/conservative.launch.py`). Gate B route, plan slowed 1.4×,
lag 0.66/0.18, pose jump guard, preset default `hover`. In the stack container:

```bash
ros2 launch team_path_following conservative.launch.py repo_root:=/root/team_ws/src/Dronehacks \
  mission_preset:=hover|first_leg|course pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom \
  base_height_on_pad_m:=<measured: OAK lens height above the floor - 0.0304>
```

Gazebo (cons03): done, 0 touches at 0.20 m, minimum clearance 0.30 m, VIO 2.4 cm RMS. That run was
RTF-starved, so its time (55 s) isn't valid; the plan is about 27 s.

**FROZEN 2026-11-08: `drone-fast-v1` = candidate 3** (`launch/fast.launch.py` / `config/tracker_fast.yaml`, drone lag gains
0.66/0.18). Justin's call: 0.25 m clearance is enough, so the strict 0.29 m bar is not required. Candidate 2 (same planner, no
chicane vias or re-timing) flew 24.8 / 25.2 s with 0 touches at 0.25 m. Candidate 3 only adds margin at the chicane. Drone command:
`ros2 launch team_path_following fast.launch.py repo_root:=/root/team_ws/src/Dronehacks mission_preset:=course pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom base_height_on_pad_m:=<measured>`.
Fly conservative first (hover, first_leg, course), then fast. Settings:
`course_v3_gateb2.yaml` (chicane vias on the centreline), sphere margin 0.35, MaxVelMag 1.2, time_scale 1.0,
clearance re-timing 0.45/0.38/1.6. Sim command:

```bash
COURSE=planner/config/course_v3_gateb2.yaml CP_VIA=5,6,7 bash path_following/sim/slot0.sh fly vio <name> \
  tracker_params:=/root/team/path_following/team_path_following/config/tracker_sim_c3.yaml \
  "planner_overrides:=MaxVelMag=1.2@DilateRadius=0.35@DilateShape=sphere"
```

Offline (kinsim): 24.4 s, minimum clearance 0.316 m. Gazebo le1–le3 (slot 0) and c3rep1–3 (slot 2) were in
flight at wind-down; results are in `path_following/runs/sim/le*` and `~/rep3_runs.log` on the VM.

Earlier candidates and strict scores (`eval_bag.py`: touches at 0.29 m, whole drone through each opening):

| Candidate | Runs | Result |
|---|---|---|
| 1 (vio20 settings) | lb1, lc1 | 19.3–19.7 s, 2–3 strict touches at maze corners and the chicane channel |
| 2 (sphere 0.35) | ld1, c2rep1 | about 19.6 s, 1–2 strict touches, only in the chicane channel |

**Still unproven:**
- **The strict 0.29 m bar.** No profile has passed it 3× in a row yet. The chicane channel has only
  0.40 m of centreline clearance, which leaves about 0.11 m for tracking plus VIO error.
- **x500 sim vs Sky Beta.** The lag 0.66/0.18 comes from Gazebo's x500. Fit the real lag from the first
  flight bags (`tools/fit_lag.py <bag> --states course`) before going faster. Under-estimating the lag
  causes wall touches.
- **Real VIO.** Gazebo showed one 9.5 m VIO jump at liftoff (vio21); the pose jump guard now aborts and
  descends. It's untested on real cuVSLAM.
- **Planning time on the Jetson** at 5 cm voxels with sphere dilation (laptop: about 3.5 s per candidate,
  3 candidates, 45 s budget).
