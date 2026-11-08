# Feeding IMU-fused VIO to PX4 on drone 12: what it would take (not deployed)

## Today's chain

- cuVSLAM: flight instance in `vslam_container`, `enable_imu_fusion: False`, set in the organizers'
  `~/skybeta/.../oakd_vslam.launch.py`.
- → `vio_level.py`: re-levels roll/pitch with PX4's attitude while the drone is still. Heading is never touched.
- → `/claude/vio_leveled/odometry`.
- → `px4_odom_bridge`, launched by the teammates' `~/team_ws/claude/mavlink_usb/bridge.launch.py` with
  `input_topic /claude/vio_leveled/odometry`, `pose_frame FRD`, output `/claude/usb/in/vehicle_visual_odometry`.
- → USB MAVLink adapter → PX4 EKF2. EKF2 fuses EV position and yaw, plus the rangefinder for height.
  No mag, no GPS.

## Two ways to switch, once the shadow has passed the runbook gates

**A. Replace the flight instance (recommended).** Never run two cuVSLAMs in flight: each costs ~40 % of a
core plus GPU, and the flight VIO's 30 Hz is what EKF2 and the tracker depend on.

- Set `enable_imu_fusion: True` in the flight cuVSLAM, together with the shadow's settings:
  - `imu_frame`: a frame published by our own static TF (`base_link → team_imu_frame`, rotation from `calib.json`).
    Never use depthai's `oak_imu_frame`, which is ~92° wrong.
  - `calibration_frequency: 200`;
  - the noise values from `shadow.sh`;
  - `visual_slam/imu` remapped to the accel-scale-corrected `/team_imu/imu` from `team_imu_prep.py`, which
    must then start with the stack.
- The flight cuVSLAM's parameters live in the organizers' tracked file, which we don't edit. The clean route is
  a team-owned copy of the launch, selected the way the teammates already select their USB launch files
  (`startup/run_competitor.sh`).
- Applying it means restarting the camera/cuVSLAM launch, and with it PX4's vision feed. Do it on the ground,
  disarmed, between flights, and check after the restart that EKF2 reports EV position/yaw fusion again.

**B. Point the bridge at the shadow.** Change `bridge.launch.py`:
`input_topic: /team_imu/visual_slam/tracking/odometry` and `status_topic: /team_imu/visual_slam/status`.
That is a one-line change in a teammate's file, but it keeps two cuVSLAMs running in flight. Only use it for a
short tethered or low hover test, and only if CPU headroom has been measured with both running.

## What changes downstream

- **Leveller.** With fusion, cuVSLAM's odom frame is gravity-aligned (check `tilt0` in the bench log), so the
  leveller's roll/pitch correction should be ~0. Keep it in the chain at first, since a re-level of ~0 is
  harmless, and watch `last_correction_deg`. Drop it once that is consistently < 0.5°. Re-levelling a fused
  estimate during stills could fight cuVSLAM's own gravity estimate.
- **Heading.** Fusion does not make heading observable. It reduces heading drift by integrating the gyro
  through fast turns and visually poor stretches, where pure VO loses rotation. The hand-carry closure-yaw
  number is the measure. EV yaw stays PX4's only heading source, so `EKF2_EV_CTRL` stays as it is.
- **Latency.** Fusion does not add a frame of latency. Keep `EKF2_EV_DELAY` and re-measure it from a flight log
  afterwards. (PX4 params are not ours to touch without Justin's OK.)
- **Tracking loss.** With fusion, cuVSLAM bridges short visual dropouts on the IMU (up to ~1 s). Without it,
  it reports `vo_state 2`. The bridge's status gating keeps working unchanged.
- **New failure mode.** A wrong IMU rotation, scale or time offset makes fusion worse than none. The sim
  showed this: a misconfigured fused VIO diverged at takeoff (see `docs/results/imu_fusion.md`). So the gates
  are mandatory:
  1. `calib_check.py` GOOD and within 3° of `calib.json`;
  2. bench drift pass;
  3. three hand-carry loops where fused beats levelled on closure h and yaw, with no tracking losses;
  4. only then, a hover with the switch.

## Rollback

- Option A: re-select the organizers' launch (marker file / wrapper) and restart the camera/cuVSLAM launch on
  the ground.
- Option B: revert the one line in `bridge.launch.py` and restart the bridge launch on the ground.
