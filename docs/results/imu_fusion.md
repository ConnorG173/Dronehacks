# cuVSLAM IMU fusion: Gazebo A/B, root cause, drone shadow, and the gyro heading fix (2026-11-08)

Branch `imu-fusion`. Sim slot 1. Drone 12 shadow kit under `deploy/imu_fusion_shadow/`.

## Bottom line

1. **Stock Isaac ROS 3.2.6 IMU fusion cannot work.** `visual_slam_impl.cpp` (`UpdatePose`) registers every IMU
   sample with the camera frame's timestamp (`latest_ts`), not its own (`imu_ts`). cuVSLAM gets ~8 samples per
   frame at one instant. The bug is unchanged in the public source from 3.0 to v3.2-15 (= the 3.2.6 binary in
   the organizers' image). Fix: `vio/patches/isaac_ros_visual_slam-3.2.6-imu-timestamps.patch`, built as an
   overlay by `vio/build_vslam.sh`.
2. **Even patched, fusion is not safe to fly.** In Gazebo, with a perfect IMU and exact timing, fused VIO is
   never better than vision-only and diverges in some runs depending on start-up timing (one flown run: error
   ~100 km with PX4 flying on it). On the drone the fused shadow jumped ~4 m while still, ~15 s after start.
3. **Heading drift is better fixed outside cuVSLAM:** `deploy/imu_fusion_shadow/heading_fix.py` takes heading
   increments from the gyro (bias learned while still) and everything else from the levelled VIO.
   - Synthetic 2 min, 720° loop: end error 1.28 m → 0.004 m.
   - To validate on the drone: hand-carry log + `analyze.py --hfix`.
4. Vision-only VIO in Gazebo is near-perfect: 2–3 cm ATE, heading drift 0.1° over 1190° of turning. The sim
   cannot reproduce the drone's ~50 cm drift, so it can only show whether fusion is stable, not whether it
   helps.

## Fusion verified active

- `Enable IMU Fusion: true`, noise parameters and `frequency` are logged at start-up.
- The node subscribes to `/drone1/sensors/imu` (250 Hz, body FLU, specific force +9.8 z at rest, sim time).
- IMU frame: static TF base_link→sim_imu, identity (PX4 x500 `imu_sensor`, no pose).
- Checked from data (`sim/imu_ab/time_offsets.py`), IMU and VIO angular rates against ground truth:
  - axis map identity;
  - IMU stamps +2 ms, image stamps 0 ms.
- Orientation sweep, all 24 axis-aligned IMU rotations, patched build: identity is best by far (0.085 m;
  next best 0.30 m). A wrong extrinsic (yaw 90°) is clearly worse, so the IMU really is used.
- Patched build, `DH_IMU_DEBUG=1` per-frame log: 8–9 IMU samples per frame, contiguous 4 ms spacing, last
  sample at the image stamp, none skipped, no registration failures.

## Gazebo: same-input replay A/B (`sim/imu_ab/replay.sh`)

Input: one recorded fusion-off Gate B flight (`rec_imu0_1`; route turns 1192°). Each row is cuVSLAM rerun on
identical images and IMU. Metrics are VIO vs `/sim/ground_truth/odom` on header stamps
(`sim/imu_ab/vio_drift.py`), aligned at takeoff (yaw-only).

| Variant | Horiz ATE rms | End error | Yaw drift at end | Tilt max |
|---|---|---|---|---|
| fusion off | 0.021 m | 0.008 m | +0.1° | 0.4° |
| fusion on, stock 3.2.6 | 4.6 m | 7.4 m | −112° | 47° |
| fusion on, stock, any of 24 IMU rotations | 0.9 – 11.7 m | – | – | – |
| fusion on, **patched**, repeats (same input) | 0.11, 0.16, 0.16, 0.69, 1.7, 1.8, 55, 1452 m | – | – | – |
| fusion on, patched, true sim noise (3 repeats) | 0.18, 4.2, 138 m | – | – | – |
| fusion on, patched, wrong extrinsic (yaw 90°) | 1.39 m | 2.06 m | +23° | 30° |

Run-to-run spread comes from start-up: identical runs stay bit-identical until one starts tracking a frame
later, then diverge chaotically 2.6–4 s after lift-off. IMU QoS depth (10 vs 200) and treating equal stamps as
strictly earlier did not help.

## Gazebo: flown A/B (`sim/imu_ab/ab.sh`, tracker on ground truth, fresh reset per run, patched build)

| Run | VIO | Outcome | Horiz ATE rms / end | Yaw drift end | Tilt drift max | RTF |
|---|---|---|---|---|---|---|
| ab2_imu0_1 | off | course done | 0.024 / 0.011 m | −0.11° | 0.42° | 1.00 |
| rec_imu0_1 | off (input recorded) | course done | 0.021 / 0.009 m | +0.05° | 0.41° | 0.82 (recording load) |
| ab2_imu1_2 | **fused** | diverged; PX4 flew on it | 96 km / 103 km | −40° | 140° | 0.85 |
| ab1_imu1_1 | fused (stock) | VIO froze during the climb, then diverged | 0.25 / 1.01 m in 7 s | −47° | 66° | 1.0 |
| ab2_imu1_1, ab2_imu0_2 | – | sim flake: never armed / never took off | – | – | – | – |

## Drone 12: fused shadow bench (patched build, OAK IMU, 2026-11-08 08:25 UTC)

- **Health:** flight VIO unchanged at 30 Hz with the shadow running (Jetson ~50 % CPU per core); shadow 30 Hz.
- **Gravity:** fused tilt within 0.3–0.6° of PX4 gravity with no leveller.
- **Start-up jump:** the fused estimate jumped ~4 m (x −3.1, z +2.7) about 15 s after start with the drone
  still. Raw and levelled VIO didn't move.
- **Disturbance:** ~3.5 min in, the drone was turned ~33° on the ground and the **flight** cuVSLAM (stock,
  what PX4 uses) jumped ~130 m. The shadow jumped at the same moment. Bench invalid from there.
- **OAK IMU (BNO086):** reads |g| = 8.63 m/s² at rest (12 % low; one-pose ×1.137 scale), and depthai's
  `oak_imu_frame` TF is ~92° wrong. Its calibration is a likely contributor to the start-up jump.
- **Next:** PX4-IMU bench, then hand-carry scored with `--hfix`.

## Reproduce

```bash
bash sim/gcp/sync.sh 1 && bash sim/imu_ab/sync_pf.sh          # this branch + path follower @45bfe51 -> slot 1
bash sim/gcp/vm.sh 1 'bash sim/imu_ab/slot1.sh build'
# build the patched node into slot 1's vio_ws (x86 sim image):
bash sim/gcp/vm.sh 1 'docker run --rm -v $PWD:/root/team skybeta-sim:$(cat ~/skybeta/VERSION) bash -lc "cd /root/team && bash vio/build_vslam.sh /root/team/vio_ws"'
bash sim/gcp/vm.sh 1 'VSLAM_PATCHED=1 nohup bash sim/imu_ab/ab.sh ab3 1 0 1 0 > /tmp/ab3.log 2>&1 &'
bash sim/gcp/vm.sh 1 'VSLAM_PATCHED=1 bash sim/imu_ab/psweep.sh runs_imu/<run>/input tag sim/imu_ab/variants_patched.txt 2'
```

`slot1.sh` is `path_following/sim/slot0.sh` with every slot-0 reference changed:
- `SIM_ID`, `--sim` and `run_sim.sh 1`;
- the competitor's `SIM_ID`;
- a guard against running outside `~/slot1`;
- a stereo + cuVSLAM check after reset (Gazebo sometimes renders one camera only).
