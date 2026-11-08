# Planner web app: integration guide

A local web app that turns a course CAD file into a flyable path file:

```
CAD file (STEP/STL/OBJ/PLY)
  -> mesh in the map frame          tools/cad_to_map.py  transform_vertices()
  -> occupied-point map (map.bin)   tools/cad_to_map.py  map_points(), write_map()
  -> config.yaml (mission + limits) web/server.py        write_config()
  -> gcopter_planner                build/gcopter_planner(.exe), retried on failure
  -> path.bin / path.csv            PX4 local NED at the pad, 100 Hz rows
```

The browser page (`index.html`, three.js) shows the course in 3D. You place the
start, checkpoints and goal, plan, and the path is drawn over the course. Click
on the path (with no placing mode active) to inspect a point: its position, time,
speed, and the shortest distance to the net and to the nearest obstacle, with a
line to each closest surface point (`/api/inspect`). Playback shows the keep-out
radius (`map.dilate`) as a sphere around the drone that turns red wherever the
path comes closer than that to a surface; those samples are also marked red on the
path. All the
real work is in `server.py` and its HTTP API, so other software can drive it
without the page. You can also skip the web layer and call the tools directly
(see the last section).

Read `../README.md` for the planner itself: config keys, validation, the
`path.bin` format and the frames. This file only covers the web layer and how to
integrate it.

## Run

```bash
# once: build the planner (../README.md "Build"); Python needs numpy + trimesh (+ cadquery for STEP)
python planner/web/server.py                 # http://127.0.0.1:8765
python planner/web/server.py --port 9000 --planner /path/to/gcopter_planner
```

- It listens on **127.0.0.1 only**. There is no authentication, so don't expose it.
- Python standard library HTTP server (`ThreadingHTTPServer`). Plans run **one at a
  time** behind a lock, and each `/api/plan` call blocks until it finishes (a few
  seconds per attempt).
- On Windows it adds the MSYS2 UCRT64 and OMPL DLL folders to `PATH` for the
  planner (`WINDOWS_DLL_DIRS` in `server.py`, set up for the team laptop). On
  Linux nothing extra is needed.
- three.js loads from unpkg (`three@0.160.0`), so the page needs internet; the API
  does not.

## Data on disk

Everything goes under `planner/out/web/` (git-ignored) and is never deleted
automatically:

```
out/web/<id>/                  one per upload; <id> = 16 hex chars
  course.<ext>                 the uploaded CAD file
  mesh.npz                     triangles in CAD units (vertices float64 N x 3, faces int64 M x 3);
                               STEP uploads also: part int32 M (part index per triangle),
                               names str P (part names), kinds str P (net / floor / obstacle)
  plan-001/, plan-002/, ...    one per /api/plan call
    map.bin                    occupied-point map (DHMAP, ../README.md "Map file")
    config.yaml                the exact planner input (re-run: gcopter_planner config.yaml --out .)
    planner.log                stdout+stderr of every attempt
    path.bin, path.csv         only if a plan passed validation
    route.txt                  RRT route, map frame (debugging)
```

## HTTP API

All bodies are JSON unless noted. Errors return HTTP 4xx/5xx with
`{"error": "<message>"}`. Every vector is in metres in the **map frame** (below)
unless noted.

### `POST /api/upload?name=<filename>`: import a course

Body: the raw file bytes (not multipart). The extension in `name` must be one of
`.step .stp .stl .obj .ply`. Limit 300 MB. STEP conversion takes about 5 s for
the competition course.

Response:
```json
{"id": "3d418c26bac492fb", "name": "UpdatedCourse.step", "triangles": 17065,
 "cadBounds": [[-3049.27, -3049.27, -6.35], [3049.27, 3049.27, 3049.27]],
 "scaleGuess": 0.001,
 "parts": [{"name": "Boulder 2", "kind": "obstacle", "triangles": 1969}, ...]}
```
`cadBounds` is in CAD units. `scaleGuess` is CAD units to metres, guessed from size
(mm 0.001, inch 0.0254, m 1). `parts` lists the STEP file's named parts and how
each is classified (see `/api/inspect`); it is empty for STL/OBJ/PLY, which carry
no part names.

```bash
curl -s --data-binary @UpdatedCourse.step "http://127.0.0.1:8765/api/upload?name=UpdatedCourse.step"
```

### `GET /api/example`: import course v3 with its mission

Same response as upload, plus `"mission"`: a complete plan request (minus `id`)
built from `config/course_v3.yaml`. It's the quickest way to see a valid request.

### `POST /api/mesh`: course mesh in the map frame (for display)

Request: `{"id": "...", "transform": Transform}`. Response
(`application/octet-stream`, little-endian):
```
uint32 nV, uint32 nF, float32[nV][3] vertices (map frame, m), uint32[nF][3] triangle indices
```

### `POST /api/plan`: build the map and plan

Request:
```json
{
  "id": "3d418c26bac492fb",
  "transform": {"scale": 0.001, "up": "z", "yaw": 0.0, "translate": [0, 0, 0]},
  "map": {"spacing": 0.04, "addFloor": false, "voxel": 0.1, "dilate": 0.3,
          "bound": [-3.2, 3.2, -3.2, 3.2, 0.0, 3.2]},
  "start": [-2.5146, 0.9144, 0.0],
  "heading": 0.0,
  "hover": 0.5,
  "checkpoints": [[2.3114, 0.4572, 0.94, 0.25, 0.30, 0.25],
                  [-0.4710, -2.2715, 0.762, 0.08, 0.08, 0.12]],
  "goal": [-2.0422, -0.7925, 0.5],
  "planner": {"maxVel": 4.0, "timeoutRRT": 0.5, "tries": 5}
}
```

| Field | Meaning | Planner key |
|---|---|---|
| `transform.scale` | CAD units to metres | (cad_to_map `--scale`) |
| `transform.up` | `"z"` or `"y"`: the CAD's up axis | (`--up`) |
| `transform.yaw` | degrees, rotation about map +z, CCW | (`--yaw-deg`) |
| `transform.translate` | metres, applied last | (`--translate`) |
| `map.spacing` | surface sample spacing (m); must be finer than `voxel` | (`--spacing`) |
| `map.addFloor` | also sample the plane z = 0 (CAD without a floor) | (`--add-floor`) |
| `map.voxel` | voxel size (m) | `VoxelWidth` |
| `map.dilate` | keep-out radius: obstacle inflation = vehicle radius + margin (m). The planner rounds it up to whole voxels (`ceil(dilate / voxel)`) and grows each occupied voxel by that as a cube | `DilateRadius` |
| `map.bound` | `xmin xmax ymin ymax zmin zmax`; outside counts as a wall. `zmax` is the flight ceiling | `MapBound` |
| `start` | the pad, **on the ground** | `StartPos` |
| `heading` | the vehicle's yaw on the pad, degrees CCW from map +x. Defines the output frame | `StartHeadingDeg` |
| `hover` | the path starts at rest at `start + (0, 0, hover)` | `HoverHeight` |
| `checkpoints` | in flight order: `[cx, cy, cz, hx, hy, hz]` axis-aligned boxes (centre, half-sizes) the path must enter | `Checkpoint` |
| `goal` | the path ends here at rest | `Goal` |
| `planner.maxVel` | speed limit (m/s) | `MaxVelMag` |
| `planner.timeoutRRT` | route-search time per leg (s) | `TimeoutRRT` |
| `planner.tries` | attempts (1 to 20); the planner is random, so a failed attempt can pass on retry | |

Every other planner key (tilt, thrust, body-rate limits, vehicle mass and drag,
optimizer weights, yaw settings, `SampleDt`) is copied from
`config/course_v3.yaml`. **The vehicle parameters there are placeholders**
(GCOPTER's demo quad).

Response:
```json
{"ok": true, "run": "plan-003", "mapPoints": 294729,
 "log": "== attempt 1/5, exit 0\n...GCOPTER plan OK\n  total time : 9.567 s ...",
 "path": {"t": [0.0, 0.01, ...], "pos": [[-2.5146, 0.9144, 0.5], ...], "speed": [0.0, ...],
          "clearance": [0.5, ...]},
 "keepOut": 0.3,
 "files": ["path.bin", "path.csv", "route.txt", "config.yaml", "map.bin", "planner.log"]}
```
- `path.pos` is in the **map frame** (for drawing). The flight file is `path.bin`
  in PX4 local NED (below).
- `path.clearance` is each sample's distance (m) to the nearest surface in the
  map, floor included (and the z = 0 plane if `addFloor`). Samples closer than
  `keepOut + spacing` get the exact point-to-triangle distance; farther ones get the
  distance to the nearest map sample point, at most `spacing / sqrt(3)` too large.
  `keepOut` echoes `map.dilate`. Adds about 0.2 s per plan.
- Geometric clearance is not the planner's voxel check. The planner's
  `dilation margin : N of M 1-ms samples inside DilateRadius` counts samples in
  dilated voxels, and those reach farther than the radius: a voxel is occupied if
  any surface sample falls in it, and the dilation is a cube. On course v3 with
  0.1 m voxels and a 0.3 m radius, one plan reported 2.6 s in the margin while only
  0.05 s was geometrically closer than 0.3 m (closest approach 0.2997 m).
- On failure, `ok` is false, and `path` and `files` are absent. `log` holds every
  attempt; the reason is on the `ERROR:` lines. Typical ones:
  - `... is inside an obstacle, its DilateRadius margin, or outside MapBound` (start
    hover point or goal blocked)
  - `checkpoint N box ... has no free point` (the whole box is blocked)
  - `no collision-free route ... within TimeoutRRT` (retry, raise `timeoutRRT`, or
    check that a gap is wider than 2 x `dilate`)
  - `trajectory misses checkpoint N` / limit violations (usually a retry fixes it)

### `POST /api/inspect`: distance from a point to the net and obstacles

Request: `{"id": "...", "transform": Transform, "point": [x, y, z]}` (map frame).
Response:
```json
{"point": [-0.471, -2.2715, 0.762], "labeled": true,
 "net":      {"dist": 0.7765, "closest": [-0.471, -3.048, 0.762], "part": "Net side south"},
 "obstacle": {"dist": 0.4568, "closest": [-0.3147, -1.8422, 0.762], "part": "C2a chicane wall (window)"}}
```
- `dist` is the exact shortest distance (m) from `point` to the course triangles
  (the same triangles the map is sampled from), `closest` the point on the surface
  that realises it, `part` the STEP part it belongs to. It is not the distance to
  the map's sample points or voxels, and ignores `dilate`.
- Parts are classified by name when a STEP file is uploaded: a name containing
  the word "net" is **net**; "floor", "landing pad", "launch mark", "guideline" or
  "approach ring" (flat floor markings) is **floor**; everything else (walls,
  dividers, barriers, tube and cradles, boulders) is an **obstacle**. The floor
  is in neither answer. `FLOOR_WORDS` in `server.py` holds the list.
- STEP part labels come from OCP's XCAF API: the named parts are meshed as one
  compound with `load_mesh`'s settings, which reproduces its triangles exactly,
  and each triangle is matched to its twin.
- `labeled` is false for STL/OBJ/PLY uploads and for STEP files uploaded before
  part labels existed (re-upload to get them). Then `net` is null and `obstacle`
  is the nearest surface of any kind, floor included, with `part` null.
- About 25 ms per request for the competition course (17k triangles).

### `GET /api/file/<id>/<run>/<name>`: download an output

`<name>` is one of `path.bin path.csv route.txt config.yaml map.bin planner.log`.

## Frames

- **Map frame:** right-handed, metres, z up, as produced by `transform` from the
  CAD: `p_map = Rz(yaw) * U * (scale * p_cad) + translate`, where `U` swaps
  y-up to z-up. Start, checkpoints, goal, bound and `path.pos` are in this frame.
- **path.bin (flight output):** PX4 local NED with its origin at `start` (on the
  ground). +x is along `heading`, +y is to its right, +z is down. With
  `c, s = cos(heading), sin(heading)` and `d = p_map - start`:
  `x = c dx + s dy`, `y = s dx - c dy`, `z = -dz`. The same 2x2 matrix converts
  back (`ned_to_map` in `server.py`).

`path.bin`: a 32-byte header (`"DHPATH\0\0"`, uint32 version 1, uint32 12 columns,
uint64 rows, float64 dt), then float64 rows of
`t, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate` (s, m, m/s, m/s^2, rad,
rad/s). The rows map one to one onto `px4_msgs/TrajectorySetpoint`. The first row
is at rest at `(0, 0, -hover)`; the last row is at rest at the goal.

## What the planner guarantees, and what it doesn't

- **Checked:** the path never enters an occupied voxel (the undilated CAD
  surface) or leaves `bound`, it enters every checkpoint box in order, and it stays
  within the speed, tilt, thrust and body-rate limits (+5%). Time spent inside the
  inflation margin is reported but allowed.
- **Checkpoints are "touch this box", not "go through this opening".** A box
  placed beside a window can be touched while the path goes around the wall, or
  **over** it if `bound.zmax` is above the wall tops. On the competition course
  the walls are 1.73 m tall and the net roof is 3.05 m. If flying over walls is not
  allowed, set `zmax` below the wall tops (1.6 m was tested), or draw boxes that
  straddle the opening.
- If a checkpoint's centre is blocked (inside the inflation margin), the planner
  aims at the nearest free voxel centre inside the box and prints a `NOTE` line.
- **Takeoff and landing are not in the path.** The consumer takes off to the first
  row, plays the rows, holds the last row, then lands. The team flight code
  (`localizer/`) does this; its simulator adapter requests the landing
  automatically.
- **Results are random.** The route search stops on a time limit, so two runs give
  different paths. Store the `path.bin` you validated; don't regenerate it at
  flight time.

## Integrating without the web layer

Every server step is a plain function or command, so other software can call
them directly:

```python
import sys; sys.path.insert(0, "planner/tools")
import cad_to_map, numpy as np
mesh = cad_to_map.load_mesh("UpdatedCourse.step")                  # trimesh (CadQuery for STEP)
v = cad_to_map.transform_vertices(mesh.vertices, 0.001, "z", 0.0, [0, 0, 0])
cad_to_map.write_map("map.bin", cad_to_map.map_points(v[np.asarray(mesh.faces)], 0.04))
```

```bash
# or the same as one command:
python planner/tools/cad_to_map.py UpdatedCourse.step --out map.bin --scale 0.001 --up z --spacing 0.04
# write a config (copy config/updated_course.yaml and edit MapFile/mission), then:
planner/build/gcopter_planner config.yaml --out outdir    # exit 0 = outdir/path.bin written
```

`gcopter_planner` exits 0 only after validation passes. Otherwise it exits 1,
prints `ERROR: ...` and leaves no `path.bin`. Before validation it prints a
machine-readable `RESULT duration=... front_ms=... corridor_ms=... opt_ms=...`
line. Retry on a nonzero exit, as the server does.
