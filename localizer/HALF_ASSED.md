# What is half-assed (fix before trusting this in the course)

This build exists to test the setpoint path to PX4 (brief option A: correct the route, not
EKF2). Every shortcut is tagged `HALF-ASSED:` in the code: `grep -rn HALF-ASSED localizer`.
The core is `localizer/core/` (shared by both adapters); "real" means `localizer/real.py`, "sim"
`localizer/sim.py`.

## Follower (core)
| Shortcut | Consequence | Real fix (brief step) |
|---|---|---|
| T is fixed at pad init. No localisation fixes are fused, so there is no T filter. | Drift of the local estimate (cuVSLAM on the drone) goes uncorrected. Fine for a bench test and a short hover; unknown over the full path. | 3-state KF on T, updated with course-frame fixes at the image stamp, plus rate limiting (8) |
| Pad init assumes the drone sits on the launch mark facing `StartHeadingDeg` (0, itself ASSUMED). There is no wide registration cross-check against the CAD. | A misplaced or mis-turned drone flies a shifted or rotated course. In the simulator the same holds for the spawn pose: check it against the launch mark (-2.51, 0.91), heading +x, with `SIM_SENSOR_EXAMPLE.py`. | wide match +-0.5 m / +-20 deg on the pad, refuse beyond 0.15 m / 5 deg (6) |
| An estimator jump, an estimator-not-ok report, a stale position, or leaving the fence only holds (HOLD the last on-path point). | Real: the pilot has to take over; there is no recovery. Sim: NAV_LAND after `land_after_hold_s` (10 s). | inflate T, hold, re-acquire with N consistent fixes (8) |
| No re-init after a run. STOPPED/DONE are final; real keeps HOLD after Offboard ends. | Restart the node for each run. | |
| HOLD after a fault re-sends p(tau), the on-path point, not the lead point. | At 1 m/s the vehicle brakes and runs up to 0.48 m past it along the path (model), then comes back. | |
| The fence is the square cage minus a margin: setpoints are clamped at 0.3 m, the vehicle faults at 0.15 m (`fence_fault_margin_m`; the course_v3 path comes within 1.4 mm of the 0.3 m line at M1/M3). | No obstacle awareness beyond the planner's path. | |

## Real adapter
| Shortcut | Consequence | Real fix (brief step) |
|---|---|---|
| z is sent as height above the floor, relying on unanchored `flight_interface` (z_ned = -z_enu) and PX4 local z ~ rangefinder height on the pad. | Needs a stack restarted with the drone on the pad. Off by the odom origin's z otherwise. | anchor or compensate z |
| `odom_to_ned_yaw_offset_rad` is assumed 0, and yaw is commanded (`command_yaw`). | If PX4's heading and the odom heading disagree, the drone yaws to the wrong heading. Measure it on the bench (status shows `expected_yaw_ned_deg` next to `px4_yaw_ned_deg`). Use `command_yaw:=false` until they agree. | measure, then set the flight_interface parameter (10) |
| Position-only setpoints (flight_interface drops velocity/acceleration). The on-path lead point p(tau + rate/`mpc_xy_p`) stands in for the feedforward, and it assumes the drone's `MPC_XY_P` = 0.95 (the v1.16 default). The path is played at `max_path_speed_mps` (1.0) by slowing time. | With the wrong Kp the vehicle runs up to about 0.35 m ahead of or behind p(tau) (model, Kp 0.7/1.3); cross-track stays under 0.1 m. | Read `MPC_XY_P` off the drone and pass `mpc_xy_p:=`. For speed, add velocity feedforward to flight_interface (proposal, not applied) |
| If the node dies, flight_interface keeps holding the last setpoint sent: the lead point, up to 1.05 m ahead on the planned path at 1 m/s. | The vehicle flies on to that point and stops there. It is on the collision-checked path, but further along than where it was. | flight_interface expiry of a feedforward setpoint |
| `COMPETITOR_PKG` is unchanged, so a stack restart brings `square_mission` back. | The adapter refuses to publish live while another publisher exists on `/skybeta/setpoint/pose`. | ask before changing `/etc/skybeta/drone.env` |

## Sim adapter
| Shortcut | Consequence |
|---|---|
| Never run against the real simulator yet (no ROS on the laptop). The full feedforward (finite TrajectorySetpoint velocity/acceleration/yawspeed) is new: no lab example sets it. | Watch the first approved run closely; the position-only fallback is to send `q` instead of `p, v, a` in `fill_trajectory_setpoint`. |
| `--i-have-team-approval` refuses unless `FLYX_LAB_ACTIVE=1`, which is only known from the lab's notebook and `onboard/services.py`. | If the terminal does not export it, the approved run refuses to start (safe failure); check with `echo $FLYX_LAB_ACTIVE`. |

## Offline model (`test/closed_loop.py`, `test/px4_model.py`)
PX4 is modelled with PositionControl's P-PID cascade at the v1.16 defaults. Assumed, not PX4
values: a 0.1 s first-order attitude lag, `MPC_Z_VEL_I_ACC` 2.0, no drag, no thrust limit, perfect
localisation, no DDS latency, a kinematic 0.7 m/s AUTO_LAND. The numbers are relative (sim vs real
adapter), not a tracking prediction for the real drone. Fix: bench and tethered logs
(`vehicle_local_position_setpoint` against `vehicle_local_position`).
