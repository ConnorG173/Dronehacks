# planner: GCOPTER path planner

A standalone program that reads a course map, a start pose, an ordered list of
checkpoints and a goal, computes the fastest dynamically feasible trajectory
through all checkpoints with [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER),
validates it, and writes a time-sampled path file in PX4 local NED. The flight
code runs it once at startup and streams the file to PX4 as
`TrajectorySetpoint` (position, velocity, acceleration, yaw, yawspeed).

It does not use ROS and never talks to PX4. It is not flight code.

```
planner/
  CMakeLists.txt         Release, -O3 (upstream flags); Eigen3 + OMPL only
  src/main.cpp           the program
  third_party/gcopter/   13 upstream headers (2 documented fixes in geo_utils.hpp); LICENSE; UPSTREAM.md
  config/course_v3.yaml  the real course (dronehacks-course-v3.step); its path is
                         ../localizer/data/course_v3_path.bin
  config/course.yaml     synthetic wall demo (the example used below)
  maps/course_v3.bin     course map (tools/cad_to_map.py, command in course_v3.yaml)
  maps/wall_demo.bin     demo map (tools/make_wall_map.py)
  tools/cad_to_map.py    CAD -> map file (laptop; numpy + trimesh, STEP needs CadQuery)
  tools/make_wall_map.py synthetic wall course (also imported by tests/run_tests.py)
  tests/                 upstream parity harness + tests
```

## Build

Needs a C++17 compiler, CMake >= 3.10, Eigen 3.3+ and OMPL (1.5 or newer).
Linux x86-64 (lab container) or ARM (Jetson Orin Nano); not Windows.

```bash
# What is already installed?
g++ --version; cmake --version
ls /usr/include/eigen3 /usr/lib/cmake/eigen3 2>/dev/null | head -2
ls /usr/share/ompl /usr/lib/*/cmake/ompl /opt/ros/*/share/ompl 2>/dev/null | head

# Install (if you have sudo; Ubuntu 22.04/24.04 and JetPack 6)
sudo apt install build-essential cmake libeigen3-dev libompl-dev

# Build
cd planner
cmake -S . -B build            # add -DCMAKE_PREFIX_PATH=/opt/ros/jazzy if OMPL came from ros-jazzy-ompl
cmake --build build -j
```

Windows laptop (for iteration only; it does not count as a target build): MSYS2 UCRT64 g++ +
Ninja, with eigen3 and boost from pacman and OMPL 1.7.0 built from source into
`C:\Users\Conno\projects\ompl-1.7.0`. In Git Bash:

```bash
export PATH=/c/msys64/ucrt64/bin:/c/Users/Conno/projects/ompl-1.7.0/lib:$PATH
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="C:/Users/Conno/projects/ompl-1.7.0;C:/msys64/ucrt64"
cmake --build build -j
./build/gcopter_planner.exe config/course_v3.yaml     # -> out/course_v3/path.bin
```

To fly a new plan, copy `out/course_v3/path.bin` over `../localizer/data/course_v3_path.bin`.
The plan is not reproducible: the RRT stops on a wall-clock limit, so every run gives a slightly
different path, and some runs fail validation (re-run them). The committed path is the one the
localizer tests pin (peak speed 2.42 m/s).

## Run

```bash
./build/gcopter_planner config/course.yaml            # writes out/path.bin, out/path.csv, out/route.txt
./build/gcopter_planner config/course.yaml --out DIR  # different output directory
```

Exit code 0 means the trajectory passed validation and the path file was
written. Any failure exits 1, prints `ERROR: ...` and leaves **no** path file
(old `path.bin`/`path.csv` are deleted before planning starts; new ones are
written to `*.tmp` and renamed). Example summary (wall demo):

```
GCOPTER plan OK
  route            : ...
  total time       : ... s (N pieces)
  max speed        : ... m/s (MaxVelMag 4.00)
  max tilt         : ... deg (MaxTiltAngle 60.2 deg)
  thrust range     : ... (MinThrust 2.00, MaxThrust 12.00)
  max body rate    : ... rad/s (MaxBdrMag 2.10, yaw excluded)
  max yaw rate     : ... deg/s (YawRateMaxDeg 90.0)
  checkpoint 1     : reached at t = ... s, box (9.000, -4.000, 1.500) +/- (0.300, 0.300, 0.300)
  dilation margin  : ... of ... 1-ms samples inside DilateRadius (no occupied voxel hit)
  solve time       : ... ms (front-end ..., corridor ..., optimizer ...)
```

## Inputs: `config/course.yaml`

Flat `Key: value` lines, `#` comments, numbers separated by spaces or commas
(`[ ]` optional). Relative paths are relative to the config file. Every key
below is required and unknown keys are an error (catches typos). Keys with
upstream names have upstream `global_planning.yaml` defaults.

| Key | Meaning |
|---|---|
| `MapFile` | occupied-point map (format below) |
| `MapBound` | `xmin xmax ymin ymax zmin zmax` of the voxel grid; outside counts as occupied |
| `VoxelWidth`, `DilateRadius` | voxel size and obstacle inflation (m); upstream 0.25 / 0.5 |
| `StartPos` | takeoff pad on the ground, map frame = PX4 local origin |
| `StartHeadingDeg` | vehicle heading at start, CCW from map +x = direction of PX4 local +x |
| `HoverHeight` | the trajectory starts at rest at `StartPos + (0, 0, HoverHeight)` |
| `Checkpoint` | `cx cy cz hx hy hz`: axis-aligned box (center, half-extents). Repeat in flight order. Must be free space |
| `Goal` | end position (at rest) |
| `TimeoutRRT` | RRT time per leg (s). 0.01 as used by upstream `plan()` (the yaml's 0.02 is never used) |
| `CorridorProgress`, `CorridorRange` | `convexCover` arguments, upstream literals 7.0 / 3.0 |
| `MaxVelMag`, `MaxBdrMag`, `MaxTiltAngle`, `MinThrust`, `MaxThrust` | `magnitudeBounds` (m/s, rad/s, rad, N, N) |
| `WeightT`, `ChiVec`, `SmoothingEps`, `IntegralIntervs`, `RelCostTol` | optimizer settings (upstream) |
| `VehicleMass`, `GravAcc`, `HorizDrag`, `VertDrag`, `ParasDrag`, `SpeedEps` | `physicalParams`. **TODO(vehicle): placeholders (upstream's demo quad), not our drone** |
| `OutDir` | output directory (overridden by `--out`) |
| `SampleDt` | path-file row spacing (s), default 0.01 |
| `YawMinSpeed`, `YawRateMaxDeg`, `YawSmoothTime` | yaw generation, see Frames |

Real vehicle values still to measure: mass, thrust range, drag terms, and
limits you trust for MaxVelMag / MaxTiltAngle / MaxBdrMag. The planner's plan is
only as feasible as these numbers.

## Map file and CAD conversion

`MapFile` is a little-endian binary list of occupied points, as upstream feeds
`VoxelMap::setOccupied` from a point cloud:

| Offset | Type | Value |
|---|---|---|
| 0 | char[8] | `DHMAP\0\0\0` |
| 8 | uint32 | version = 1 |
| 12 | uint32 | 0 (reserved) |
| 16 | uint64 | N points |
| 24 | float32[N][3] | x, y, z in metres, map frame |

NaN/inf points are skipped and points outside `MapBound` are ignored. Every
voxel touched by a point is occupied, then dilated by
`ceil(DilateRadius / VoxelWidth)` voxels. Only surfaces need sampling.

`tools/cad_to_map.py` (laptop; numpy + trimesh; STEP also needs CadQuery)
samples a CAD mesh (STL/OBJ/PLY, or STEP) on a barycentric grid finer than the
voxels and writes the map:

```bash
python3 tools/cad_to_map.py course.stl --out maps/course.bin \
    --scale 0.001 --up y --yaw-deg 0 --translate 0 0 0 --spacing 0.08 [--add-floor]
```

CAD-to-map transform for each CAD vertex `p`:

```
p_map = Rz(yaw_deg) * U * (scale * p) + translate
  scale       CAD units -> metres (0.001 for mm)
  U           --up z: identity;  --up y: (x, y, z) -> (x, -z, y), i.e. CAD +y becomes map +z
  Rz          rotation about map +z, counter-clockwise seen from above
  translate   metres, applied last
```

Choose these so the map frame is right-handed with **z up** (GCOPTER models
gravity along -z) and the floor at the bottom. Then read `StartPos`,
checkpoints and the goal off the same frame. Include the floor (`--add-floor`
if the CAD has none) so it gets a dilation margin. The script prints the point
bounds; `MapBound` must contain the whole flying volume.
`tools/make_wall_map.py` writes the synthetic demo/test course.

## What the program does

The pipeline is upstream `gcopter/src/global_planning.cpp` (commit `e0444f6`)
without ROS. `src/main.cpp` marks each step with the upstream line number:

1. `VoxelMap(MapBound / VoxelWidth, ...)`, `setOccupied` for every point,
   `dilate(ceil(DilateRadius / VoxelWidth))`.
2. Start hover point, checkpoint centers and goal must be free
   (`query == 0`, as upstream checks the goal).
3. `sfc_gen::planPath` (OMPL InformedRRT*) **leg by leg**: start -> C1 -> C2 ->
   ... -> goal; each leg must reach its target. `route.txt` (all legs joined)
   is written here.
4. `getSurf`, then `convexCover(leg, ..., 7.0, 3.0)` and `shortCut` **per leg**.
   Not on the combined route: `shortCut` keeps the fewest polytopes that still
   connect its start to its end, so on a combined route it deletes the
   polytopes of any detour to a checkpoint (observed on the wall course:
   15+ polytopes contain C1 before `shortCut`, none overlaps it after).
5. **Checkpoint boxes (our only extension).** Each box is one 6-row H-polytope
   (`h0 x + h1 y + h2 z + h3 <= 0`, upstream's format) placed between its two
   legs: `[leg 1 polytopes] box 1 [leg 2 polytopes] box 2 ... [last leg]`.
   `shortCut` keeps each leg's first and last polytope, which contain the leg's
   endpoints (the checkpoint centers), so each box normally overlaps both
   neighbours. Overlap means the intersection holds a 1 cm ball
   (`geo_utils::overlap`, the margin `shortCut` uses); if either side does not
   overlap, the program fails with a clear error. With no checkpoints there
   is one leg and steps 3-6 are exactly upstream's.
6. `gcopter.setup(WeightT, iniPVA, finPVA, hPolys, INFINITY, ...)` and
   `optimize` **once** on the whole corridor. `lengthPerPiece = INFINITY` gives
   one polynomial piece per polytope, so one piece lies in each box and the
   solver picks the speed through it. Start and end states are at rest.
   The line `RESULT duration=... front_ms=... corridor_ms=... opt_ms=...` is
   printed here, before validation (used by the parity test).
7. **Validation.** The corridor and the limits are only soft penalties, so the
   trajectory is sampled every 1 ms. It must stay inside `MapBound`, never
   enter an occupied (undilated) voxel, enter every checkpoint box in order,
   and keep speed, tilt, thrust and body rate (GCOPTER's flatness map, yaw
   excluded) within the `magnitudeBounds` plus 5% for the normal soft-penalty
   overshoot. Time inside the dilation margin is reported but allowed. Any
   violation (or NaN): exit 1, no path file.

The 13 headers in `third_party/gcopter` are upstream's except `geo_utils.hpp`,
which has a 1-word compiler fix and a bug fix in vertex enumeration (bogus
vertices from degenerate hull faces caused upstream's NaN crashes, most
optimizer failures and some bad trajectories). Diff, evidence and checksums are
in `UPSTREAM.md`. Their only dependencies are Eigen and OMPL (`sfc_gen.hpp`).
The RRT front end stops on a time limit, so two runs can give slightly
different routes and durations.

## Output: `out/path.bin`, `out/path.csv`, `out/route.txt`

`path.bin`, little-endian:

| Offset | Type | Value |
|---|---|---|
| 0 | char[8] | `DHPATH\0\0` |
| 8 | uint32 | version = 1 |
| 12 | uint32 | columns = 12 |
| 16 | uint64 | rows R |
| 24 | float64 | `SampleDt` (s) |
| 32 | float64[R][12] | `t, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate` |

Rows are at `t = 0, dt, 2 dt, ...` plus a final row at exactly the total
duration. Units: s, m, m/s, m/s^2, rad, rad/s. Positions, velocities,
accelerations and yaw are in PX4 local NED (below), ready for
`TrajectorySetpoint.position / velocity / acceleration / yaw / yawspeed`.

```python
import numpy as np
raw = open("out/path.bin", "rb").read()
assert raw[:8] == b"DHPATH\0\0"
rows = np.frombuffer(raw, "<f8", offset=32).reshape(-1, 12)
```

`path.csv` is the same table with a header line, for plotting. `route.txt` is
the RRT route in map coordinates (debugging and the parity test).

The first row is at rest at `(0, 0, -HoverHeight)` with yaw 0: the flight code
should take off, settle at that point, then stream rows by elapsed time and hold
the last row (at rest at the goal) afterwards.

## Frames

**Map frame:** course coordinates in metres, right-handed, z up.

**PX4 local NED** (output): origin at `StartPos` (the pad, on the ground),
`+x` = forward along `StartHeadingDeg`, `+y` = right of that, `+z` = down. With
`psi0 = StartHeadingDeg` and `d = p_map - StartPos`:

```
x_ned =  cos(psi0) * d_x + sin(psi0) * d_y
y_ned =  sin(psi0) * d_x - cos(psi0) * d_y
z_ned = -d_z
```

Velocity and acceleration use the same rotation (no offset). A map heading
`theta` (CCW from map +x) becomes `yaw_ned = psi0 - theta` (clockwise from local
+x, wrapped to [-pi, pi]); `yaw_rate_ned = -dtheta/dt`.

Worked example: `StartPos = (2, 3, 0)`, `StartHeadingDeg = 90` (the vehicle faces
map +y).

| Map | NED | Meaning |
|---|---|---|
| point (2, 5, 1.5) | (2, 0, -1.5) | 2 m ahead, 1.5 m up |
| point (3, 3, 0) | (0, 1, 0) | 1 m to the right, on the ground |
| velocity (0, 4, 0) | (4, 0, 0) | 4 m/s forward |
| heading 0 deg (facing map +x) | yaw +90 deg | facing right |

This assumes PX4's local `+x` is the vehicle's forward direction at startup
(as with an external-vision frame initialised at the pad). If PX4's local frame
is north-aligned instead (magnetometer/GPS), set `StartHeadingDeg` to the map
direction of north and expect a non-zero initial yaw error; check this in the
flight code before streaming.

**Yaw** (GCOPTER plans none): heading = direction of horizontal velocity;
below `YawMinSpeed` the last heading is held (the start heading until the
vehicle moves); unwrapped; limited to `YawRateMaxDeg`; then a centered moving
average over `YawSmoothTime` (window shrinks at the ends, so the first row is
exactly the start heading). `yaw_rate` is the central difference of the smoothed
yaw. GCOPTER's tilt and thrust limits are independent of yaw; the body-rate
limit `MaxBdrMag` does not include yaw rate.

## Tests

```bash
python3 tests/run_tests.py build          # --runs N for the end-to-end parity runs (default 20)
```

Python standard library only. It runs:

1. **Parity.** `tests/upstream_demo.cpp` is upstream's `mapCallBack()` +
   `plan()` with ROS removed, upstream's yaml values hard-coded, built from the
   same headers and flags. On a mockamap-style Perlin map (upstream launch
   parameters, seed 1024; written from scratch, mockamap is GPLv3):
   (a) our route fed to both corridor+optimizer stages must give the
   bit-identical duration; (b) N end-to-end runs each, medians of duration
   (within 5%) and solve time (within 25%) compared.
2. **Checkpoints** on the wall course: without checkpoints the path does not
   pass checkpoint 1; with it, the path enters the box (checked again from
   `path.csv`); with two checkpoints on opposite sides of the wall, both are
   entered in order.
3. **Impossible checkpoint** inside the wall: non-zero exit, and stale path
   files are removed.
4. **Path file format**: header, row count, bin == CSV, first/last rows.

## Limitations

- Vehicle parameters are placeholders (TODO(vehicle) in `course.yaml`).
- Checkpoint boxes are axis-aligned in the map frame and assumed obstacle-free;
  a box overlapping an obstacle is caught by validation, not avoided.
- The route is random (RRT with a time limit); re-run if a plan fails
  validation, or raise `TimeoutRRT` for long legs.
- The plan assumes PX4 tracks it well. Check tracking error in simulation
  before trusting the margins on hardware.
