# Localization results

One row per run. Newest first within each section. Commit = branch `localization`.

## Offline closed loop (no Gazebo): `tools/loc_dev/closed_loop.py`

What it is:
- **Truth:** the planner's course_v3 path (path following's `runs/course_v3_v1.6/path.bin`,
  18.7 m), played back with a 2.5 s climb from the pad. Tilt comes from the path's accelerations.
- **VIO:** the true motion integrated with injected **yaw drift (°/m)** and **scale error**, from an
  odom origin 2.2 m and 23° away from the pad.
- **Depth:** 160×100 images at 10 Hz, raycast from the course wall grid (walls extruded to 1.7 m,
  nets transparent, floor). 55 % fill, 1 % range noise.
- **PX4 topics:** attitude, landed, armed, dist_bottom, all faked from truth.
- **Under test:** the real `loc_fuser` (`enable_correction:=true`) and `wall_matcher`
  (`source:=depth`, 5 Hz), built from this branch.
- **Comparison:** "raw (M1)" is the same VIO with the milestone-1 anchor only.

What it is not: the depth is ideal-geometry (no stereo artefacts, no low-texture holes beyond random
dropout), the walls are a 2D extrusion, VIO drift is a smooth model, and there are no cuVSLAM
failures. It tests the estimator, the gating, the frames and the timing end to end, not the stereo
or VIO.

Commits 0fa830f..91a7bfa (`dropout` and `vio_jump` on 6fe84b0+; the estimator is identical, later
commits fixed jump handling and added sanity checks). Status counts are 10 Hz samples after takeoff.

| scenario | path m | corrected RMS / max / end (m) | raw (M1) RMS / max / end (m) | yaw RMS corr / raw (°) | fixes acc / rej / reacq | status OK / WARN / ERROR (air, 10 Hz) | err > 0.3 m while OK | err > 3σ |
|---|---|---|---|---|---|---|---|---|
| mild: yaw 0.3°/m, scale 1.01, speed ×0.7 | 18.6 | 0.046 / 0.121 / 0.024 | 0.191 / 0.313 / 0.265 | 1.0 / 3.5 | 69 / 0 / 0 | 236 / 0 / 0 | 0 % | 0 % |
| base: yaw 1.0°/m, scale 1.02, speed ×0.7 | 18.6 | 0.100 / 0.212 / 0.025 | 0.631 / 1.030 / 0.901 | 2.8 / 11.5 | 68 / 3 / 1 | 216 / 20 / 0 | 0 % | 0 % |
| strong_neg: yaw -1.0°/m, scale 0.98, speed ×0.7 | 18.6 | 0.096 / 0.257 / 0.039 | 0.571 / 0.940 / 0.774 | 2.4 / 11.5 | 69 / 0 / 0 | 232 / 4 / 0 | 0 % | 0 % |
| fast: yaw 0.5°/m, scale 1.01, speed ×1.0 | 18.6 | 0.067 / 0.134 / 0.036 | 0.314 / 0.504 / 0.435 | 1.6 / 5.9 | 54 / 0 / 0 | 170 / 1 / 0 | 0 % | 0 % |
| dropout: yaw 0.5°/m, scale 1.01, speed ×0.7, dropout 4:1.0 | 18.6 | 0.055 / 0.129 / 0.023 | 0.316 / 0.504 / 0.435 | 1.5 / 5.9 | 64 / 0 / 0 | 219 / 7 / 10 | 0 % | 0 % |
| vio_jump: yaw 0.5°/m, scale 1.01, speed ×0.7, VIO jump 6:0.6 | 18.6 | 0.243 / 0.667 / 0.019 | 1.001 / 1.668 / 1.401 | 5.4 / 14.1 | 53 / 0 / 0 | 157 / 17 / 62 | 0 % | 0 % |
| m1_baseline: yaw 0.5°/m, scale 1.01, speed ×0.7, correction off | 18.6 | 0.309 / 0.504 / 0.435 | 0.308 / 0.504 / 0.435 | 5.8 / 5.8 | None / None / None | 236 / 0 / 0 | 45 % | 0 % |

Reading it:
- With the correction on, the error at the end of the course is 2–4 cm in every scenario, against
  0.26–1.4 m for the anchor-only (M1) estimate. RMS is 3–6× lower.
- **Honesty:** no run had error above 0.3 m while status said OK. With the correction on, no error
  exceeded 3σ of the published covariance.
- **The M1 limitation:** without fixes the fuser cannot see its own drift. It reported OK during 45 %
  of the samples where error was above 0.3 m (M1 row). M1's covariance grows with distance, but it
  does not WARN on sigma, by design.
- **Dropout (1 s of no VIO and no depth):** ERROR during the gap, no false jump on resume (yaw-rate
  scaled threshold), and the correction continues afterwards.
- **VIO teleport (0.6 m + 10°, like a cuVSLAM reset):** ERROR at once. The correction is widened and
  re-acquired from 3 fixes, and ERROR clears after about 6 s. The max error (0.67 m) falls inside the
  ERROR period.

## Gazebo slot 1 (real stack: cuVSLAM on rendered stereo, PX4 SITL, path following's tracker)

`tools/loc_dev/slot1.sh fly gt <name>`: the tracker flies the course on **ground truth**, and
localization runs with correction plus wall matcher, so its errors cannot affect the flight.
Errors are /team/odom_map vs `/sim/ground_truth/odom` (`tools/loc_eval.py`, `tools/loc_dev/bag_loc_debug.py`).

| run | course textures | outcome | odom_map ATE h RMS / max | checkpoint errors (m) | status | notes |
|---|---|---|---|---|---|---|
| gt01 | 6 tiled | no takeoff | – | – | ERROR on the pad | anchor froze on the pad: PX4 says not-landed after arming with throttle centred; bit-identical sim VIO then read as "frozen". **Fixed:** freeze needs real motion; static pose alone is WARN |
| gt03 | 6 tiled | tracker held mid-course | 0.30 / 0.36 | 0.02, 0.02, 0.07, 0.19, 0.33 | ERROR 72 % | real cuVSLAM jump of 0.31 m in one frame latched ERROR, so the tracker held. **Fixed:** jumps are absorbed into the anchor (pose continuous, WARN 3 s) |
| gt04 | 6 tiled | **mission done** (64.5 s) | 0.22 / 0.49 | 0.02, 0.04, 0.07, 0.06, 0.32 | OK 59 % / WARN 41 % / ERROR 0 | a cuVSLAM burst of 10 frames jumping up to 2.8 m and 141°, absorbed. 8 % of samples had error > 0.3 m while status OK, so the correction is **not** trustworthy on this course |

| gt05 | per-part (3dedfc0) | mission done | 0.126 / 0.23 (**raw VIO 0.090**) | corr 0.05, 0.11, 0.07, 0.20, 0.15 / raw 0.03, 0.02, 0.04, 0.06, 0.19 | WARN 99.9 % | cuVSLAM clean (1 step of 0.13 m). **The correction was worse than raw VIO**: the matcher landed 12–24 cm off even with the prior on truth. Cause: the nets were modelled as invisible, but the cameras see them, so the matcher slid scans onto interior walls |
| gt08 | per-part | mission done (47 s; RTF ≈ 0.3, invalid for dynamics) | **0.058 / 0.12** (raw VIO 0.084) | corr 0.04, 0.04, 0.03, 0.11, 0.10 / raw 0.05, 0.05, 0.06, 0.13, 0.10 | OK 93 % / WARN 7 % / ERROR 0; never error > 0.3 m while OK | after the net fix (half-strength net surfaces): 219 of 443 matcher attempts accepted, match error p50 7.9 cm. **The correction beats raw VIO** (RMS −31 %), but the window error (≈ 0.10 m) is not below raw yet. One run, at low RTF |

The matcher on the gt05 scans (37 dumped scans, prior on truth):

| net model | accepted | match error p50 |
|---|---|---|
| invisible (old) | 4/37 | 24 cm |
| weak (default now) | 25/37 | 4 cm |
| surface | 33/37 | 5 cm |

`surface` is unsafe if the real nets are see-through: 8 of 160 synthetic fixes were overconfident with
invisible nets and room returns. `weak` gave 1 of 320 (d² 13). Only `invisible` and `neutral`
gave none, and both are bad on the sim scans. Parameter: `wallgrid.build_wallgrid(net_mode=...)`.

The matcher diagnostic on gt04 (416 attempts with ground truth):
- When the prior was within 5 cm of truth, the scan scored **0.956 at the prior**, which means stereo
  depth, camera extrinsics and map alignment are right.
- Low scores come from wrong priors, after cuVSLAM jumped on the repetitive textures. In that
  situation the matcher can lock onto the wrong structure (match error p50 0.29 m). The coordinator
  re-textured the course (sim-and-plans 3dedfc0) because the tiles caused those cuVSLAM jumps.

## Matcher property tests (synthetic scans on the course grid, `src/team_loc_matcher/test`)

| test | result |
|---|---|
| 400 random free poses (5 seeds × 40, dense and edge-only scans, net/room returns, random prior offsets ±0.3 m / ±6°) | accepted 103/200 dense, 56/200 edge-only; **0 accepted fixes outside their own 3-dof 99 % covariance bound** (worst d² 5.5) after the covariance fix (`n_eff_cov` 25). The prototype setting (one cap of 50) had 2/400 overconfident fixes, d² up to 48 |
| known poses (6) incl. the launch pad | within 4 cm and 2°, inside their covariance |
| corridor (no wall ends in view) | along-axis σ > 0.15 m, across < 0.05 m: a ridge, not a confident wrong fix |
| one-bay alias (1.12 m) | refused by the filter's Mahalanobis gate |
| one wall moved 10–20 cm | fix within 5 cm when other walls are in view |
| match runtime | 7–12 ms per match on a laptop core (17 × 17 × 17 search, ~200 wall + 600 free points) |

## Unit and smoke tests

- `bash tools/loc_dev/build_test.sh`: 22 gtests (anchor, VIO health, status, covariance, correction
  EKF: convergence, outlier, re-acquire, aliases, slew) and 47 matcher pytests. All pass.
- `tools/loc_dev/smoke_fuser.py`, which uses fake cuVSLAM and PX4 topics with the stack started off the pad:
  - course pose exact (0.000 mm); frame `course`; twist copied; TF `course → map` published
  - status OK in flight, ERROR on `vo_state` 2 and on stale VIO
  - round-trip latency of our publish → `/team/odom_map` → us: p50 1.3 ms, p95 2.5 ms (two DDS hops into Python)

## Drone 12 measurements (read-only, stack 0.3.2)

| what | value |
|---|---|
| cuVSLAM at rest, 15 min (bench bag `drift_still`) | max \|dxy\| 3.3 cm, \|dz\| 4.3 cm, \|dyaw\| 0.52°; 0 steps > 5 cm; vo_state 1 throughout |
| VIO arrival latency (receive − header stamp) | p50 60–160 ms, p95 70–340 ms depending on other load (prototype bench) |
| SGBM 512×320, 96 disp (prototype bench, next to cuVSLAM) | 36 ms p50, 49 p95; stereo fill 53 % |
| prototype BEV per frame | 37 ms p50 |
| load with the stack | CPU 30–40 % of 6 cores, GPU 1–10 %, RAM 3.3 / 7.6 GB |

## Not measured yet

- Gazebo slot 1: VIO vs ground truth, the matcher on real (rendered stereo) images, the dim tube.
- On the drone: loc_fuser CPU and latency next to the stack, the matcher's rate and CPU with SGBM,
  the pad wide-match, the camera distortion question (`camera_info` has D = 0).
