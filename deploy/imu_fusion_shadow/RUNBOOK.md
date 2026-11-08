# Shadow VIO with IMU fusion on drone 12: runbook

Kit: `~/team_ws/team_staging/imu_fusion/` on skybeta-12 (source: branch `imu-fusion`,
`deploy/imu_fusion_shadow/`). The directory carries `COLCON_IGNORE`.

**What it is:** a second cuVSLAM with IMU fusion on. It reads the same stereo images and the OAK IMU and
publishes only under `/team_imu/visual_slam/*`. It publishes no TF, and nothing in the flight chain reads it.
The flight chain is cuVSLAM → leveller (`/claude/vio_leveled/odometry`) → bridge → USB adapter → PX4,
and none of it is touched: no launch is stopped or restarted, and no parameter, file or PX4 setting changes.

**Rules:**
- Run it only while **disarmed and landed**. `shadow.sh start` refuses otherwise. Its logger (`--guard`) kills
  the shadow the moment PX4 reports armed, not landed, or stale telemetry (> 3 s).
- It costs about 40 % of one CPU core, ~0.7 GB RAM and some GPU. That is why it is a bench tool only.
- Stop it before handing the drone back: `shadow.sh stop`.
- Ask Justin before every start. The drone is shared with a teammate's Codex sessions.
- Rule 6.4: this kit reads no competition-run data, and its logs must not be used to tune competition runs.

## Files

| File | What |
|---|---|
| `shadow.sh` | `check` (read-only preconditions), `start [oak\|px4]`, `stop`, `status` (host side; uses `docker exec` into `skybeta-stack`) |
| `team_imu_prep.py` | `/oak/imu/data` → `/team_imu/imu`: accel scale fix, frame `team_imu_frame`, drops non-increasing stamps. `px4` mode reads PX4's `sensor_combined` instead |
| `team_imu_log.py` | read-only logger and guard: flight VIO raw and levelled, shadow, PX4 attitude → `logs/<stamp>/track.jsonl` |
| `analyze.py` | scores a log: loop closure (position and yaw), tilt vs PX4, rates, dropouts |
| `calib_check.py` | read-only IMU→camera rotation, time offset and accel scale, measured against the flight VIO (hand rotation) |
| `calib.json` | IMU rotation, translation and accel scale. Seeded from the teammates' `~/team_ws/claude/imu_calib.json` (copied, not linked) |

## Step 0: preconditions (read-only, no permission needed)

```bash
ssh skybeta
bash ~/team_ws/team_staging/imu_fusion/shadow.sh check
```

Expect `OK: disarmed and landed`, flight VIO ≈ 30 Hz with `vo_state 1`, and more than 1.2 GB of memory available.

## Step 1: calibration check (ASK FIRST: it runs a ROS node that only subscribes)

```bash
docker exec -it skybeta-stack bash -c 'source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/local_setup.bash; \
  cd /root/team_ws/team_staging/imu_fusion && python3 calib_check.py --secs 30'
```

1. Hold the drone still for the first 3 s.
2. Then rotate it smoothly by hand about roll, pitch and yaw, several times each, keeping the cameras on a
   textured scene.

Accept the result when:
- the rotation differs from `calib.json` by less than ~3°;
- `fit quality: GOOD`;
- |time offset| < 10 ms.

If the rotation is off by more than that, rerun with `--write` (keeps `calib.json.bak`). If the time offset
is ≥ 10 ms, note it: cuVSLAM 3.2 has no IMU time-offset parameter, so `team_imu_prep.py` would have to
shift the stamps.

## Step 2: bench (ASK FIRST: GPU/CPU)

```bash
bash ~/team_ws/team_staging/imu_fusion/shadow.sh start oak     # prints the IMU params and "Enable IMU Fusion: true"
tail -f ~/team_ws/team_staging/imu_fusion/logs/<stamp>_oak/logger.log   # every 2 s: rate, vo, tiltErr, dyaw, moved
```

Leave the drone untouched on the floor for 5 minutes, then run `shadow.sh stop` and
`python3 analyze.py logs/<stamp>_oak`.

Pass when:
- `imu` drifts less than 2 cm and 0.3° in yaw;
- `imu` tilt error is < 1° throughout;
- flight `raw` VIO rate is still ~30 Hz.

The logger prints the raw rate. If the raw rate drops below 28 Hz with the shadow on, stop: the shadow
competes too much.

## Step 3: hand-carry loop (ASK FIRST)

1. Tape a start mark on the floor and the drone's heading next to it (a line along the body x axis).
2. `shadow.sh start oak`, then leave the drone still on the mark for 20 s.
3. Carry it at flight height (~1 m) around a loop like the course, about 2 min long, turning a lot: several
   full turns in total, both directions, ~600–700° of total rotation like the real flights. Keep the cameras
   pointed forward and walk at about flight speed (0.5–1 m/s).
4. Put it back exactly on the mark at the start heading, then hold still for 20 s.
5. `shadow.sh stop`, then `python3 analyze.py logs/<stamp>_oak`.
6. Repeat 3 times, alternating the loop direction.

Reading the result: `closure h` / `z` / `yaw` are pure drift for each estimator, because the drone ends where
it began. `lev` is what PX4 flew on in the two real flights, which ended ~50 cm off. Fusion is worth wiring to
PX4 only if `imu` beats `lev` on closure yaw and closure h in every repeat, without tracking losses
(`lost%`) or jumps.

## Optional: PX4 IMU instead of the OAK's

`shadow.sh start px4` feeds PX4's `/fmu/out/sensor_combined` (FRD → FLU, base_link axes, uXRCE time sync)
instead of the BNO086. Check first that it is published at all in the current USB setup:
`shadow.sh check` shows the flight topics only, so run `ros2 topic hz /fmu/out/sensor_combined` in the
container. It also delivers only ~77 Hz, and its clock differs from the camera's (time sync, not one
device clock). The OAK IMU is the better default.

## Parameters (in `shadow.sh`)

OAK: `calibration_frequency 200` (the driver delivers 199.5 Hz), gyro noise 0.0012 rad/s/√Hz,
gyro random walk 0.0002, accel noise 0.02 m/s²/√Hz, accel random walk 0.01. These are inflated over the
BNO086 datasheet for frame vibration and its on-chip dynamic calibration. Override with
`SHADOW_NOISE="gn grw an arw" shadow.sh start oak`.
