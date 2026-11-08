#!/usr/bin/env python3
"""Web front end for gcopter_planner: upload a course CAD file, place start,
checkpoints and goal in a 3D viewer, plan, and see the path over the course.

    python planner/web/server.py            # then open http://127.0.0.1:8765

Laptop tool, standard library + the planner's own tools (numpy, trimesh, CadQuery
for STEP). It listens on localhost only. Each upload and plan is kept under
planner/out/web/<id>/ (git-ignored). It does not fly anything: the output is
the same path.bin the planner CLI writes.
"""
import argparse
import json
import math
import os
import re
import secrets
import shutil
import subprocess
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PLANNER = os.path.dirname(HERE)
REPO = os.path.dirname(PLANNER)
sys.path.insert(0, os.path.join(PLANNER, "tools"))
import cad_to_map  # noqa: E402

WORK = os.path.join(PLANNER, "out", "web")
TEMPLATE = os.path.join(PLANNER, "config", "course_v3.yaml")  # source of every key the UI does not set
EXAMPLE_STEP = os.path.join(REPO, "dronehacks-course-v3.step")
# Windows laptop build (README "Build"): the planner needs these DLL folders on PATH.
WINDOWS_DLL_DIRS = [r"C:\msys64\ucrt64\bin", r"C:\Users\Conno\projects\ompl-1.7.0\lib"]
MAX_UPLOAD = 300 * 1024 * 1024
CAD_EXTS = (".step", ".stp", ".stl", ".obj", ".ply")
OUTPUT_FILES = ("path.bin", "path.csv", "route.txt", "config.yaml", "map.bin", "planner.log")

plan_lock = threading.Lock()  # one planner run at a time
planner_exe = None


# ---------- config template ----------

def read_config(path):
    """course yaml -> list of (key, value-string), comments stripped, order kept."""
    out = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = line.split(":", 1)
            out.append((k.strip(), v.strip()))
    return out


def nums(s):
    return [float(x) for x in re.split(r"[\s,\[\]]+", s) if x]


def fmt(v):
    if isinstance(v, str):
        return v
    return "[" + ", ".join("%.6g" % x for x in v) + "]" if isinstance(v, (list, tuple)) else "%.6g" % v


def write_config(path, settings):
    """Template keys in template order, UI settings replacing them; Checkpoint lines from the UI."""
    lines = ["# written by planner/web/server.py; other keys from config/course_v3.yaml"]
    for k, v in read_config(TEMPLATE):
        if k == "Checkpoint":
            continue
        if k == "Goal":  # keep checkpoints before Goal, as in the template
            lines += ["Checkpoint: " + fmt(cp) for cp in settings["Checkpoint"]]
        lines.append("%s: %s" % (k, fmt(settings[k]) if k in settings else v))
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def example_mission():
    """The mission in config/course_v3.yaml, in the UI's terms."""
    cfg = read_config(TEMPLATE)
    get = lambda k: nums(dict(cfg)[k])
    return {
        "transform": {"scale": 0.001, "up": "z", "yaw": 0.0, "translate": [0, 0, 0]},
        "map": {"spacing": 0.04, "addFloor": False, "voxel": get("VoxelWidth")[0],
                "dilate": get("DilateRadius")[0], "bound": get("MapBound")},
        "start": get("StartPos"), "heading": get("StartHeadingDeg")[0], "hover": get("HoverHeight")[0],
        "checkpoints": [nums(v) for k, v in cfg if k == "Checkpoint"],
        "goal": get("Goal"),
        "planner": {"maxVel": get("MaxVelMag")[0], "timeoutRRT": get("TimeoutRRT")[0], "tries": 5},
    }


# ---------- course files ----------

def course_dir(cid):
    if not re.fullmatch(r"[0-9a-f]{16}", cid or ""):
        raise ValueError("bad course id")
    d = os.path.join(WORK, cid)
    if not os.path.isdir(d):
        raise ValueError("unknown course id (re-upload)")
    return d


def import_course(src_path, name):
    """Copy a CAD file into a new course dir and cache its triangles (CAD units)."""
    cid = secrets.token_hex(8)
    d = os.path.join(WORK, cid)
    os.makedirs(d)
    ext = os.path.splitext(name)[1].lower()
    cad = os.path.join(d, "course" + ext)
    shutil.copyfile(src_path, cad)
    mesh = cad_to_map.load_mesh(cad)
    v = np.asarray(mesh.vertices, dtype=np.float64)
    np.savez(os.path.join(d, "mesh.npz"), vertices=v, faces=np.asarray(mesh.faces, dtype=np.int64))
    ext_max = float((v.max(axis=0) - v.min(axis=0)).max())
    # Guess units from size: a course is metres to tens of metres across.
    guess = 0.001 if ext_max > 200 else (0.0254 if ext_max > 60 else 1.0)
    return {"id": cid, "name": name, "triangles": int(len(mesh.faces)),
            "cadBounds": [v.min(axis=0).tolist(), v.max(axis=0).tolist()], "scaleGuess": guess}


def map_vertices(cid, t):
    m = np.load(os.path.join(course_dir(cid), "mesh.npz"))
    v = cad_to_map.transform_vertices(m["vertices"], float(t["scale"]), t["up"], float(t["yaw"]),
                                      [float(x) for x in t["translate"]])
    return v, m["faces"]


def mesh_blob(cid, t):
    """Map-frame mesh: uint32 nV, uint32 nF, float32[nV*3], uint32[nF*3]."""
    v, f = map_vertices(cid, t)
    return (np.array([len(v), len(f)], "<u4").tobytes() + v.astype("<f4").tobytes()
            + f.astype("<u4").tobytes())


# ---------- planning ----------

def find_planner(explicit):
    if explicit:
        return explicit
    for name in ("gcopter_planner.exe", "gcopter_planner"):
        p = os.path.join(PLANNER, "build", name)
        if os.path.isfile(p):
            return p
    sys.exit("gcopter_planner not built: see planner/README.md 'Build' (or pass --planner PATH)")


def ned_to_map(rows, start, heading_deg):
    """Inverse of the planner's map -> NED (README 'Frames'); the 2x2 rotation is its own inverse."""
    c, s = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    x, y, z = rows[:, 1], rows[:, 2], rows[:, 3]
    return np.stack([c * x + s * y + start[0], s * x - c * y + start[1], -z + start[2]], axis=1)


def plan(req):
    d = course_dir(req["id"])
    t, mp, pl = req["transform"], req["map"], req["planner"]
    runs = [n for n in os.listdir(d) if n.startswith("plan-")]
    run = "plan-%03d" % (len(runs) + 1)
    rd = os.path.join(d, run)
    os.makedirs(rd)

    v, f = map_vertices(req["id"], t)
    pts = cad_to_map.map_points(v[f], float(mp["spacing"]), bool(mp["addFloor"]))
    cad_to_map.write_map(os.path.join(rd, "map.bin"), pts)

    settings = {
        "MapFile": "map.bin", "MapBound": [float(x) for x in mp["bound"]],
        "VoxelWidth": float(mp["voxel"]), "DilateRadius": float(mp["dilate"]),
        "StartPos": [float(x) for x in req["start"]], "StartHeadingDeg": float(req["heading"]),
        "HoverHeight": float(req["hover"]),
        "Checkpoint": [[float(x) for x in cp] for cp in req["checkpoints"]],
        "Goal": [float(x) for x in req["goal"]],
        "TimeoutRRT": float(pl["timeoutRRT"]), "MaxVelMag": float(pl["maxVel"]), "OutDir": ".",
    }
    write_config(os.path.join(rd, "config.yaml"), settings)

    env = dict(os.environ)
    if os.name == "nt":
        env["PATH"] = os.pathsep.join([p for p in WINDOWS_DLL_DIRS if os.path.isdir(p)] + [env.get("PATH", "")])
    log, ok = [], False
    tries = max(1, min(20, int(pl["tries"])))
    for i in range(tries):  # the RRT is random: a failed run can pass on retry
        r = subprocess.run([planner_exe, "config.yaml", "--out", "."], cwd=rd, env=env,
                           capture_output=True, text=True, timeout=300)
        log.append("== attempt %d/%d, exit %d\n%s%s" % (i + 1, tries, r.returncode, r.stdout, r.stderr))
        if r.returncode == 0:
            ok = True
            break
    log = "\n".join(log)
    with open(os.path.join(rd, "planner.log"), "w", encoding="utf-8") as fh:
        fh.write(log)

    res = {"ok": ok, "run": run, "log": log, "mapPoints": int(len(pts))}
    if ok:
        raw = open(os.path.join(rd, "path.bin"), "rb").read()
        rows = np.frombuffer(raw, "<f8", offset=32).reshape(-1, 12)
        p = ned_to_map(rows, settings["StartPos"], settings["StartHeadingDeg"])
        res["path"] = {"t": rows[:, 0].tolist(), "pos": p.round(4).tolist(),
                       "speed": np.linalg.norm(rows[:, 4:7], axis=1).round(3).tolist()}
        res["files"] = [n for n in OUTPUT_FILES if os.path.isfile(os.path.join(rd, n))]
    return res


# ---------- HTTP ----------

class Handler(BaseHTTPRequestHandler):
    def send(self, code, body, ctype="application/json", extra=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n > MAX_UPLOAD:
            raise ValueError("upload too large")
        return self.rfile.read(n)

    def do_GET(self):
        u = urlparse(self.path)
        try:
            if u.path in ("/", "/index.html"):
                return self.send(200, open(os.path.join(HERE, "index.html"), "rb").read(), "text/html; charset=utf-8")
            if u.path == "/api/example":
                info = import_course(EXAMPLE_STEP, os.path.basename(EXAMPLE_STEP))
                info["mission"] = example_mission()
                return self.send(200, info)
            m = re.fullmatch(r"/api/file/([0-9a-f]{16})/(plan-\d{3})/([\w.]+)", u.path)
            if m and m.group(3) in OUTPUT_FILES:
                p = os.path.join(course_dir(m.group(1)), m.group(2), m.group(3))
                if os.path.isfile(p):
                    return self.send(200, open(p, "rb").read(), "application/octet-stream",
                                     {"Content-Disposition": 'attachment; filename="%s"' % m.group(3)})
            self.send(404, {"error": "not found"})
        except Exception as e:  # report to the page instead of dropping the connection
            self.send(500, {"error": "%s: %s" % (type(e).__name__, e)})

    def do_POST(self):
        u = urlparse(self.path)
        try:
            if u.path == "/api/upload":
                name = os.path.basename(parse_qs(u.query).get("name", ["course.step"])[0])
                if not name.lower().endswith(CAD_EXTS):
                    return self.send(400, {"error": "need one of " + ", ".join(CAD_EXTS)})
                tmp = os.path.join(WORK, "upload-" + secrets.token_hex(4))
                with open(tmp, "wb") as fh:
                    fh.write(self.body())
                try:
                    return self.send(200, import_course(tmp, name))
                finally:
                    os.remove(tmp)
            req = json.loads(self.body() or b"{}")
            if u.path == "/api/mesh":
                return self.send(200, mesh_blob(req["id"], req["transform"]), "application/octet-stream")
            if u.path == "/api/plan":
                with plan_lock:
                    return self.send(200, plan(req))
            self.send(404, {"error": "not found"})
        except Exception as e:
            self.send(500, {"error": "%s: %s" % (type(e).__name__, e)})

    def log_message(self, fmt_, *args):
        sys.stderr.write("%s\n" % (fmt_ % args))


def main():
    global planner_exe
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--planner", help="gcopter_planner executable (default planner/build/)")
    args = ap.parse_args()
    planner_exe = find_planner(args.planner)
    os.makedirs(WORK, exist_ok=True)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print("planner web UI: http://127.0.0.1:%d  (planner %s, work dir %s)" % (args.port, planner_exe, WORK))
    srv.serve_forever()


if __name__ == "__main__":
    main()
