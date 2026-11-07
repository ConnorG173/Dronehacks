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
