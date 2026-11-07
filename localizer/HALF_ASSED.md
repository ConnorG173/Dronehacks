# What is half-assed (fix before trusting this in the course)

This build exists to test the setpoint path to PX4 (brief option A: correct the route, not
EKF2). Every shortcut is tagged `HALF-ASSED:` in the code: `grep -rn HALF-ASSED localizer`.

## path_streamer
| Shortcut | Consequence | Real fix (brief step) |
|---|---|---|
| T is fixed at pad init. No `course_matcher` fixes are fused, so there is no T filter. | cuVSLAM drift through the course goes uncorrected. Fine for a bench test and a short hover; unknown over the full path. | 3-state KF on T, updated with fixes at the image stamp, plus rate limiting (8) |
| Pad init assumes the drone sits on the launch mark facing `StartHeadingDeg` (0, itself ASSUMED). There is no wide registration cross-check against the CAD. | A misplaced or mis-turned drone flies a shifted or rotated course. | wide match ±0.5 m / ±20° on the pad, refuse beyond 0.15 m / 5° (6) |
| An odom jump, a `vo_state` change, stale odom, or leaving the fence only freezes the stream (HOLD the last setpoint). | The pilot has to take over. There is no recovery. | inflate T, hold, re-acquire with N consistent fixes (8) |
| No re-init after a run. Offboard ending puts the node in HOLD for good. | Restart the node for each run. | |
| z is sent as height above the floor, relying on unanchored `flight_interface` (z_ned = −z_enu) and PX4 local z ≈ rangefinder height on the pad. | Needs a stack restarted with the drone on the pad. Off by the odom origin's z otherwise. | anchor or compensate z |
| `odom_to_ned_yaw_offset_rad` is assumed 0, and yaw is commanded (`command_yaw`). | If PX4's heading and the odom heading disagree, the drone yaws to the wrong heading. Measure it on the bench (status shows `expected_yaw_ned_deg` next to `px4_yaw_ned_deg`). Use `command_yaw:=false` until they agree. | measure, then set the flight_interface parameter (10) |
| Position-only setpoints (flight_interface drops velocity/acceleration). The on-path lead point p(τ + rate/`mpc_xy_p`) stands in for the feedforward, and it assumes the drone's `MPC_XY_P` = 0.95 (the v1.16 default). The path is played at `max_path_speed_mps` (1.0) by slowing time. | With the wrong Kp the vehicle runs up to about 0.3 m ahead of or behind p(τ) (sim, Kp 0.7/1.3); the path clock slows, cross-track stays under 0.1 m. Above about 1.5 m/s corner cutting grows (0.34 m at 2.5 m/s). | Read `MPC_XY_P` off the drone and set `mpc_xy_p`. For speed, add velocity feedforward to flight_interface (proposal, not applied) |
| HOLD after a fault re-sends p(τ), the on-path point, not the lead point. | At 1 m/s the vehicle brakes and runs up to 0.48 m past it along the path (sim), then comes back. | |
| If path_streamer dies, flight_interface keeps holding the last setpoint sent: the lead point, up to 1.05 m ahead on the planned path at 1 m/s. | The vehicle flies on to that point and stops there. It is on the collision-checked path, but further along than where it was. | flight_interface expiry of a feedforward setpoint |
| `tools/sim_streamer.py` / `test/px4_model.py`: PX4 is modelled with PositionControl's P–PID cascade at the v1.16 defaults. Assumed, not PX4 values: a 0.1 s first-order attitude lag, `MPC_Z_VEL_I_ACC` 2.0, no drag, no thrust limit, perfect localisation, no DDS latency. | The numbers are relative (old vs new), not a tracking prediction for the real drone. | Bench and tethered logs (`vehicle_local_position_setpoint` against `vehicle_local_position`) |
| The fence only clamps setpoints and holds when the vehicle leaves the cage−0.3 m box. | No obstacle awareness beyond the planner's path. | |

## course_matcher
| Shortcut | Consequence |
|---|---|
| Passive: `/localizer/fix` is not consumed by anything. | Logging / validation only. |
| Nearest odom sample and *latest* attitude, not interpolated at the image stamp. | Up to ~17 ms of odom skew; tilt skew during fast manoeuvres. |
| No Mahalanobis or `vo_state` gate (there is no filter). Prior = T0 ∘ odom. | Alias protection relies on the ±0.4 m window only. **Do not feed these fixes into anything yet.** |
| Stereo only. Depth Anything is not wired in (it was slower on the bench). | The A/B in step 5 still needs doing. |
| Matcher thresholds (`n_eff_max`, `min_wall_pts`, σ) come from synthetic scans only. | Calibrate against the taped marks on course bags (steps 4–5). |
| Single process, ~5 Hz, no profiling on the Jetson yet. | |

## Not done at all
- Step 4–5: course bags and offline replay (needs a hand-carry through the course).
- Pre-flight as-built survey.
- `COMPETITOR_PKG` is unchanged, so a stack restart brings `square_mission` back. `path_streamer` refuses to publish live while another publisher exists on `/skybeta/setpoint/pose`.
