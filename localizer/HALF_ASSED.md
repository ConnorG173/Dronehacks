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
| Position-only setpoints, so the path's velocity/acceleration feedforward is dropped (flight_interface limitation). The path is played at `max_path_speed_mps` (1.0) by slowing time. | Lag behind the carrot. The carrot pauses beyond 0.6 m of error. | |
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
