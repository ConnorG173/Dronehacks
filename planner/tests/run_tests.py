#!/usr/bin/env python3
"""Planner tests. Python standard library only; never flies anything.

    python3 tests/run_tests.py [BUILD_DIR] [--runs N]

1. Parity with upstream (tests/upstream_demo.cpp) on a mockamap-style map:
   a) identical front-end route -> corridor + optimizer must give the
      bit-identical trajectory duration;
   b) end-to-end, N runs each (the RRT front end is time-limited, so results
      vary run to run): medians of duration and solve time are compared.
2. Checkpoint tests on the synthetic wall course (tools/make_wall_map.py).
3. Impossible checkpoint (inside the wall) fails and leaves no path file.
4. path.bin header/rows agree with path.csv and the start/goal states.
5. Regression for the DroneHacks sfc_gen.hpp fix: a random course that used to segfault in
   OMPL's InformedRRTstar destructor now plans (skipped without numpy).
6. config/course_v3.yaml still plans (up to 3 attempts; no attempt may crash).
"""
import math
import os
import statistics
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import make_wall_map  # noqa: E402

args = [a for a in sys.argv[1:] if not a.startswith("--")]
RUNS = int(sys.argv[sys.argv.index("--runs") + 1]) if "--runs" in sys.argv else 20
if "--runs" in sys.argv:
    args.remove(str(RUNS))
BUILD = os.path.abspath(args[0]) if args else os.path.join(ROOT, "build")
EXE = ".exe" if os.name == "nt" else ""
PLANNER = os.path.join(BUILD, "gcopter_planner" + EXE)
UPSTREAM = os.path.join(BUILD, "upstream_demo" + EXE)
WORK = os.path.join(ROOT, "out", "tests")

# upstream gcopter/config/global_planning.yaml, in our key names
UPSTREAM_PARAMS = """
VoxelWidth: 0.25
DilateRadius: 0.5
TimeoutRRT: 0.01
CorridorProgress: 7.0
CorridorRange: 3.0
MaxVelMag: 4.0
MaxBdrMag: 2.1
MaxTiltAngle: 1.05
MinThrust: 2.0
MaxThrust: 12.0
VehicleMass: 0.61
GravAcc: 9.8
HorizDrag: 0.70
VertDrag: 0.80
ParasDrag: 0.01
SpeedEps: 0.0001
WeightT: 20.0
ChiVec: 1.0e+4 1.0e+4 1.0e+4 1.0e+4 1.0e+5
SmoothingEps: 1.0e-2
IntegralIntervs: 16
RelCostTol: 1.0e-5
OutDir: out
SampleDt: 0.01
YawMinSpeed: 0.3
YawRateMaxDeg: 90.0
YawSmoothTime: 0.5
"""

failures = []


def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        failures.append(msg)


def run(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    return p.returncode, p.stdout


def result(output):
    for line in output.splitlines():
        if line.startswith("RESULT "):
            return {k: v for k, v in (kv.split("=") for kv in line.split()[1:])}
    raise RuntimeError("no RESULT line in output:\n" + output)


def vec(v):
    return " ".join(repr(float(x)) for x in v)


def write_cfg(name, map_file, map_bound, start, hover, goal, checkpoints=()):
    d = os.path.join(WORK, name)
    os.makedirs(d, exist_ok=True)
    lines = ["MapFile: " + map_file, "MapBound: " + vec(map_bound), "StartPos: " + vec(start),
             "StartHeadingDeg: 0", "HoverHeight: %r" % hover, "Goal: " + vec(goal)]
    lines += ["Checkpoint: " + vec(c) for c in checkpoints]
    path = os.path.join(d, "course.yaml")
    with open(path, "w") as f:
        f.write("\n".join(lines) + UPSTREAM_PARAMS)
    return path, os.path.join(d, "out")


def read_csv(path):
    with open(path) as f:
        next(f)
        return [[float(x) for x in line.split(",")] for line in f]


def to_ned(p, pad=(0.0, 0.0, 0.0), heading_deg=0.0):
    c, s = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    d = [p[i] - pad[i] for i in range(3)]
    return (c * d[0] + s * d[1], s * d[0] - c * d[1], -d[2])


def entry_times(rows, boxes):
    """First time each box (map frame, heading 0, pad at origin) is entered, in order."""
    times, k = [], 0
    for r in rows:
        if k == len(boxes):
            break
        c, h = to_ned(boxes[k][:3]), boxes[k][3:]
        if all(abs(r[1 + i] - c[i]) <= h[i] for i in range(3)):
            times.append(r[0])
            k += 1
    return times


def ever_inside(rows, box):
    c, h = to_ned(box[:3]), box[3:]
    return any(all(abs(r[1 + i] - c[i]) <= h[i] for i in range(3)) for r in rows)


def parity():
    print("\n== 1. Parity with upstream demo pipeline ==")
    os.makedirs(WORK, exist_ok=True)
    map_file = os.path.join(WORK, "perlin_map.bin")
    code, out = run([UPSTREAM, "map", map_file])
    assert code == 0, out
    sg = {l.split()[0]: [float(x) for x in l.split()[1:]] for l in out.splitlines()}
    start, goal = sg["start"], sg["goal"]
    print("  map: %d points, start %s, goal %s" % (sg["points"][0], start, goal))
    cfg, outdir = write_cfg("parity", map_file, (-25, 25, -25, 25, 0, 5), start, 0.0, goal)

    # Our planner prints RESULT right after optimizing, before its own validation
    # (upstream has none), so a run our validator rejects still counts as a solve.
    upstream_cmd = [UPSTREAM, "plan", map_file] + [repr(x) for x in start + goal]

    # a) identical route: our route fed to upstream's corridor + optimizer.
    # Some RRT routes make upstream's getShortestPath return NaN, which trips an
    # Eigen assertion in gcopter.hpp setInitial (upstream and ours alike), so retry.
    for _ in range(10):
        code, out = run([PLANNER, cfg, "--out", outdir])
        if "RESULT " in out:
            break
    ours = result(out)
    route = os.path.join(outdir, "route_fixed.txt")
    os.replace(os.path.join(outdir, "route.txt"), route)
    code_up, out_up = run(upstream_cmd + [route])
    assert code_up == 0, out_up
    up = result(out_up)
    print("  fixed route: ours duration %s s, upstream %s s%s" % (
        ours["duration"], up["duration"], "" if code == 0 else " (ours rejected by validation)"))
    check(ours["duration"] == up["duration"], "identical route -> bit-identical trajectory duration")

    # b) end-to-end
    dur = {"ours": [], "upstream": []}
    ms = {"ours": [], "upstream": []}
    fails = {"ours": 0, "upstream": 0}
    rejected = []
    for _ in range(RUNS):
        for who, cmd in (("ours", [PLANNER, cfg, "--out", outdir]), ("upstream", upstream_cmd)):
            code, out = run(cmd)
            if "RESULT " not in out:
                fails[who] += 1
                continue
            r = result(out)
            dur[who].append(float(r["duration"]))
            ms[who].append(float(r["front_ms"]) + float(r["corridor_ms"]) + float(r["opt_ms"]))
            if who == "ours" and code != 0:
                rejected.append(next(l for l in out.splitlines() if l.startswith("ERROR")))
    print("  end-to-end, %d runs each        ours        upstream" % RUNS)
    print("    %-28s %-11.3f %.3f" % ("median duration (s)", statistics.median(dur["ours"]),
                                     statistics.median(dur["upstream"])))
    print("    %-28s %-11.3f %.3f" % ("min duration (s)", min(dur["ours"]), min(dur["upstream"])))
    print("    %-28s %-11.1f %.1f" % ("median solve time (ms)", statistics.median(ms["ours"]),
                                     statistics.median(ms["upstream"])))
    print("    %-28s %-11d %d" % ("solver failures", fails["ours"], fails["upstream"]))
    print("    %-28s %-11d n/a" % ("rejected by our validation", len(rejected)))
    for msg in sorted(set(m.split(" at t=")[0] for m in rejected)):
        print("      %dx %s" % (sum(m.startswith(msg) for m in rejected), msg))
    rel = abs(statistics.median(dur["ours"]) / statistics.median(dur["upstream"]) - 1)
    check(rel < 0.05, "end-to-end median duration within 5%% (%.1f%%)" % (100 * rel))
    rel_ms = abs(statistics.median(ms["ours"]) / statistics.median(ms["upstream"]) - 1)
    check(rel_ms < 0.25, "end-to-end median solve time within 25%% (%.1f%%)" % (100 * rel_ms))
    check(fails["ours"] <= fails["upstream"] + 1, "ours fails to solve no more often than upstream")


def checkpoints():
    print("\n== 2. Checkpoints on the wall course ==")
    os.makedirs(WORK, exist_ok=True)
    map_file = os.path.join(WORK, "wall.bin")
    make_wall_map.write_map(map_file, make_wall_map.wall_course_points())
    bound, start, hover, goal = make_wall_map.MAP_BOUND, (0.0, 0.0, 0.0), 1.5, (11.0, 0.0, 1.5)
    c1 = (9.0, -4.0, 1.5, 0.3, 0.3, 0.3)   # behind the wall, far off the straight line
    c2 = (2.5, 3.5, 2.0, 0.3, 0.3, 0.3)    # back in front of the wall

    # The straight line start-hover -> goal is y = 0, z = 1.5; it misses c1 by 3.7 m.
    check(abs(c1[1]) - c1[4] > 3.0, "checkpoint 1 is far from the straight start->goal line")

    cfg, out = write_cfg("baseline", map_file, bound, start, hover, goal)
    code, log = run([PLANNER, cfg, "--out", out])
    check(code == 0, "no checkpoints: plan succeeds")
    if code == 0:
        check(not ever_inside(read_csv(os.path.join(out, "path.csv")), c1),
              "no checkpoints: path does not pass checkpoint 1 by itself")

    cfg, out = write_cfg("one_checkpoint", map_file, bound, start, hover, goal, [c1])
    code, log = run([PLANNER, cfg, "--out", out])
    print("\n".join("    " + l for l in log.splitlines()))
    check(code == 0, "one checkpoint: plan succeeds")
    if code == 0:
        t = entry_times(read_csv(os.path.join(out, "path.csv")), [c1])
        check(len(t) == 1, "one checkpoint: path.csv enters the box (t = %s)" % t)

    cfg, out = write_cfg("two_checkpoints", map_file, bound, start, hover, goal, [c1, c2])
    code, log = run([PLANNER, cfg, "--out", out])
    print("\n".join("    " + l for l in log.splitlines() if "checkpoint" in l or "total" in l))
    check(code == 0, "two checkpoints (through the opening three times): plan succeeds")
    if code == 0:
        t = entry_times(read_csv(os.path.join(out, "path.csv")), [c1, c2])
        check(len(t) == 2, "two checkpoints: path.csv enters both boxes in order (t = %s)" % t)

    print("\n== 3. Impossible checkpoint ==")
    bad = (5.2, -3.0, 1.5, 0.3, 0.3, 0.3)  # center inside the wall
    cfg, out = write_cfg("impossible", map_file, bound, start, hover, goal, [bad])
    os.makedirs(out, exist_ok=True)
    for f in ("path.bin", "path.csv"):  # stale files from an earlier run must be removed
        open(os.path.join(out, f), "w").write("stale")
    code, log = run([PLANNER, cfg, "--out", out])
    print("    " + log.strip().replace("\n", "\n    "))
    check(code != 0, "checkpoint inside an obstacle: non-zero exit")
    check(not any(os.path.exists(os.path.join(out, f)) for f in ("path.bin", "path.csv")),
          "checkpoint inside an obstacle: no path file left")

    print("\n== 4. Path file format ==")
    out = os.path.join(WORK, "one_checkpoint", "out")
    with open(os.path.join(out, "path.bin"), "rb") as f:
        data = f.read()
    magic, version, ncols, nrows, dt = struct.unpack("<8sIIQd", data[:32])
    rows = [struct.unpack_from("<12d", data, 32 + 96 * i) for i in range(nrows)]
    csv = read_csv(os.path.join(out, "path.csv"))
    check(magic == b"DHPATH\0\0" and version == 1 and ncols == 12 and dt == 0.01, "header fields")
    check(len(data) == 32 + 96 * nrows and nrows == len(csv), "row count matches file size and CSV")
    check(max(abs(a - b) for r, c in zip(rows, csv) for a, b in zip(r, c)) < 1e-4, "bin rows equal CSV rows")
    first, last = rows[0], rows[-1]
    check(first[0] == 0.0 and max(abs(x) for x in first[1:3] + first[4:7]) < 1e-9 and abs(first[3] + hover) < 1e-9,
          "first row: t=0 at (0, 0, -HoverHeight) NED, zero velocity")
    check(abs(first[10]) < 1e-9, "first row: yaw = start heading (0)")
    g = to_ned(goal)
    check(max(abs(last[1 + i] - g[i]) for i in range(3)) < 1e-6 and max(abs(x) for x in last[4:7]) < 1e-6,
          "last row: at goal (NED), zero velocity")
    check(all(rows[i + 1][0] > rows[i][0] for i in range(nrows - 1)), "time strictly increasing")


def straight_legs():
    """Random course seed 3 (path_following/tools/make_random_map.py) segfaulted every run before
    the sfc_gen.hpp fix: OMPL's InformedRRTstar destructor after a 2-state (straight) solution."""
    print("\n== 5. Random course, seed 3 (regression: OMPL destructor segfault) ==")
    gen = os.path.join(os.path.dirname(ROOT), "path_following", "tools", "make_random_map.py")
    d = os.path.join(WORK, "random_seed3")
    code, log = run([sys.executable, gen, d, "--seed", "3",
                     "--template", os.path.join(ROOT, "config", "course_v3.yaml")])
    if code != 0:
        print("  SKIP  random map generator unavailable (needs numpy):", log.strip().splitlines()[-1:])
        return
    codes = []
    for _ in range(3):
        code, log = run([PLANNER, os.path.join(d, "course.yaml"), "--out", os.path.join(d, "out")])
        codes.append(code)
        if code == 0:
            print("\n".join("    " + l for l in log.splitlines() if "total" in l))
            break
    check(all(c >= 0 and c != 139 for c in codes), "random seed 3: no attempt crashed (exits %s)" % codes)
    check(codes[-1] == 0, "random seed 3: plans within 3 attempts (exits %s)" % codes)

def course_v3():
    print("\n== 6. Real course config still plans ==")
    cfg = os.path.join(ROOT, "config", "course_v3.yaml")
    out = os.path.join(WORK, "course_v3")
    codes = []
    for _ in range(3):
        code, log = run([PLANNER, cfg, "--out", out])
        codes.append(code)
        if code == 0:
            print("\n".join("    " + l for l in log.splitlines() if "total" in l))
            break
    check(all(c >= 0 and c != 139 for c in codes), "course_v3: no attempt crashed (exits %s)" % codes)
    check(codes[-1] == 0, "course_v3: plans within 3 attempts (exits %s)" % codes)


if __name__ == "__main__":
    for exe in (PLANNER, UPSTREAM):
        if not os.path.exists(exe):
            sys.exit("missing %s; build first (see README)" % exe)
    parity()
    checkpoints()
    straight_legs()
    course_v3()
    print("\n%s: %d failure(s)" % ("FAILED" if failures else "ALL PASSED", len(failures)))
    sys.exit(1 if failures else 0)
