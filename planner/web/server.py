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
import time
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
    f = np.asarray(mesh.faces, dtype=np.int64)
    arrays = {"vertices": v, "faces": f}
    parts = []
    if ext in (".step", ".stp"):
        try:
            part, names = step_part_labels(cad, v[f])
            arrays.update(part=part, names=np.array(names), kinds=np.array([part_kind(n) for n in names]))
            parts = [{"name": n, "kind": part_kind(n), "triangles": int((part == i).sum())} for i, n in enumerate(names)]
        except Exception as e:  # inspection then falls back to "nearest surface of any kind"
            print("part labels unavailable for %s: %s: %s" % (name, type(e).__name__, e), file=sys.stderr)
    np.savez(os.path.join(d, "mesh.npz"), **arrays)
    ext_max = float((v.max(axis=0) - v.min(axis=0)).max())
    # Guess units from size: a course is metres to tens of metres across.
    guess = 0.001 if ext_max > 200 else (0.0254 if ext_max > 60 else 1.0)
    return {"id": cid, "name": name, "triangles": int(len(f)),
            "cadBounds": [v.min(axis=0).tolist(), v.max(axis=0).tolist()], "scaleGuess": guess,
            "parts": parts}


# ---------- part labels and point inspection ----------

# Flat floor markings (pad rings, launch mark, guideline) count as floor, not obstacles.
FLOOR_WORDS = ("floor", "landing pad", "launch mark", "guideline", "approach ring")


def part_kind(name):
    """'net', 'floor' or 'obstacle', from a STEP part name."""
    n = name.lower()
    if re.search(r"\bnets?\b", n):
        return "net"
    return "floor" if any(w in n for w in FLOOR_WORDS) else "obstacle"


def step_part_labels(path, tris):
    """Part index per triangle of `tris` (load_mesh's triangles, CAD units) and the part names.

    Reads the STEP's named parts with OCP's XCAF API and meshes them as one compound
    with load_mesh's settings, which gives the same triangles (BRepMesh's relative
    deflection depends on the whole shape, so meshing parts one by one would not).
    Each triangle is then matched to its twin by centroid.
    """
    from OCP.BRep import BRep_Builder, BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS, TopoDS_Compound
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from scipy.spatial import cKDTree

    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("XCAF could not read the STEP file")
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        a = TDataStd_Name()
        return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), a) else ""

    parts = []  # (name, located shape) per leaf part

    def walk(label):
        ref = label
        if st.IsReference_s(label):
            ref = TDF_Label()
            st.GetReferredShape_s(label, ref)
        if st.IsAssembly_s(ref):
            kids = TDF_LabelSequence()
            st.GetComponents_s(ref, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i))
        else:
            parts.append((name(label) or name(ref), st.GetShape_s(label)))

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i))

    comp = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(comp)
    for _, shape in parts:
        builder.Add(comp, shape)
    BRepMesh_IncrementalMesh(comp, 0.1, True, 0.1, True)  # = load_mesh's tolerance / angularTolerance

    cents, owner = [], []
    for k, (_, shape) in enumerate(parts):
        ex = TopExp_Explorer(shape, TopAbs_FACE)
        while ex.More():
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(TopoDS.Face_s(ex.Current()), loc)
            if tri is not None:
                trsf = loc.Transformation()
                nodes = np.array([tri.Node(i).Transformed(trsf).Coord() for i in range(1, tri.NbNodes() + 1)])
                idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
                cents.append(nodes[idx].mean(axis=1))
                owner += [k] * len(idx)
            ex.Next()
    dist, nearest = cKDTree(np.concatenate(cents)).query(tris.mean(axis=1))
    size = float(np.ptp(tris.reshape(-1, 3), axis=0).max())
    if dist.max() > 1e-6 * size:  # STL float32 rounding is ~1e-7 of the size
        raise ValueError("part tessellation does not match the mesh (max centroid gap %.3g)" % dist.max())
    return np.array(owner, dtype=np.int32)[nearest], [n for n, _ in parts]


def closest_on_triangles(p, tris):
    """Closest point to p on each triangle (T x 3 x 3) -> (T x 3 points, T distances).

    Vectorised form of Ericson, Real-Time Collision Detection, 5.1.5. Degenerate
    triangles give NaN; their edges belong to neighbouring triangles anyway.
    """
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    ab, ac = b - a, c - a
    dot = lambda u, w: np.einsum("ij,ij->i", u, w)
    ap, bp, cp = p - a, p - b, p - c
    d1, d2 = dot(ab, ap), dot(ac, ap)
    d3, d4 = dot(ab, bp), dot(ac, bp)
    d5, d6 = dot(ab, cp), dot(ac, cp)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    col = lambda x: x[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        den = va + vb + vc
        q = a + ab * col(vb / den) + ac * col(vc / den)  # inside the face
        # Regions in reverse order of Ericson's tests, so the earlier test wins.
        q = np.where(col((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0)),
                     b + (c - b) * col((d4 - d3) / ((d4 - d3) + (d5 - d6))), q)        # edge BC
        q = np.where(col((vb <= 0) & (d2 >= 0) & (d6 <= 0)), a + ac * col(d2 / (d2 - d6)), q)  # edge AC
        q = np.where(col((d6 >= 0) & (d5 <= d6)), c, q)                                 # vertex C
        q = np.where(col((vc <= 0) & (d1 >= 0) & (d3 <= 0)), a + ab * col(d1 / (d1 - d3)), q)  # edge AB
        q = np.where(col((d3 >= 0) & (d4 <= d3)), b, q)                                 # vertex B
        q = np.where(col((d1 <= 0) & (d2 <= 0)), a, q)                                  # vertex A
    return q, np.linalg.norm(q - p, axis=1)


def path_clearance(path, tris, pts, cutoff, add_floor):
    """Distance (m) from each path point to the nearest surface in the map, floor included.

    First the distance to the nearest map sample point (`pts`), which overestimates
    the true distance by at most spacing / sqrt(3). Every point closer than `cutoff`
    (keep-out radius + spacing, so no point inside the keep-out radius is missed)
    then gets the exact distance to the triangles whose bounding box is in reach.
    """
    from scipy.spatial import cKDTree
    clear = cKDTree(pts).query(path)[0]
    lo, hi = tris.min(axis=1), tris.max(axis=1)
    for i in np.flatnonzero(clear < cutoff):
        near = np.all((lo <= path[i] + cutoff) & (hi >= path[i] - cutoff), axis=1)
        clear[i] = np.nanmin(closest_on_triangles(path[i], tris[near])[1]) if near.any() else np.inf
    if add_floor:  # the z = 0 plane map_points() adds, which is not in `tris`
        clear = np.minimum(clear, np.abs(path[:, 2]))
    return clear


def inspect(req):
    """Shortest distances from a map-frame point to the nearest net and obstacle surfaces."""
    p = np.array([float(x) for x in req["point"]])
    v, f = map_vertices(req["id"], req["transform"])
    tris = v[f]
    m = np.load(os.path.join(course_dir(req["id"]), "mesh.npz"))
    labeled = "part" in m.files
    if labeled:
        kind = m["kinds"][m["part"]]
        groups = {"net": kind == "net", "obstacle": kind == "obstacle"}
    else:  # no part names: one "obstacle" answer over every surface, floor included
        groups = {"net": None, "obstacle": np.ones(len(tris), bool)}
    res = {"point": p.tolist(), "labeled": labeled}
    for key, mask in groups.items():
        res[key] = None
        if mask is None or not mask.any():
            continue
        q, dist = closest_on_triangles(p, tris[mask])
        i = int(np.nanargmin(dist))
        res[key] = {"dist": round(float(dist[i]), 4), "closest": q[i].round(4).tolist(),
                    "part": str(m["names"][m["part"][np.flatnonzero(mask)[i]]]) if labeled else None}
    return res


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


# Live progress of the running plan, polled by the page (GET /api/progress). One plan at a time (plan_lock).
progress = {"active": False}
progress_lock = threading.Lock()


def set_progress(**kw):
    with progress_lock:
        if kw.pop("reset", False):
            progress.clear()
            progress.update(active=True, t0=time.time(), stage="map", detail="", attempt=0, tries=0,
                            leg=0, legs=0, lines=[], attempts=[])
        progress.update(kw)
        progress["elapsed"] = round(time.time() - progress.get("t0", time.time()), 2)


def get_progress():
    with progress_lock:
        p = json.loads(json.dumps(progress))
    if p.get("active"):
        p["elapsed"] = round(time.time() - p["t0"], 2)
    return p


def run_planner(rd, env, attempt):
    """One gcopter_planner run, streaming its STAGE lines into progress. Returns (exit code, output)."""
    proc = subprocess.Popen([planner_exe, "config.yaml", "--out", "."], cwd=rd, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
    killer = threading.Timer(300, proc.kill)      # same 300 s limit as before
    killer.start()
    out = []
    try:
        for line in proc.stdout:
            lg = re.match(r"LEG (\d+) (.*)", line)
            if lg:                                    # one leg's RRT route: [[x, y, z], ...]
                pts = [[float(v) for v in q.split()] for q in lg.group(2).split(";") if q.strip()]
                with progress_lock:
                    progress["routes"] = progress.get("routes", []) + [pts]
                continue                              # long line: kept out of the log
            out.append(line)
            m = re.match(r"STAGE (\w+) ?(.*)", line)
            with progress_lock:
                progress["lines"] = (progress.get("lines", []) + [line.rstrip()])[-14:]
                if m:
                    stage, rest = m.group(1), m.group(2)
                    progress["stage"], progress["detail"] = stage, rest
                    lm = re.match(r"(\d+)/(\d+) (.*)", rest) if stage == "route" else None
                    if lm:
                        progress["leg"], progress["legs"], progress["detail"] = int(lm.group(1)), int(lm.group(2)), lm.group(3)
                elif line.startswith("ERROR:"):
                    progress["detail"] = line[6:].strip()
        proc.wait()
    finally:
        killer.cancel()
    return proc.returncode, "".join(out)


def plan(req):
    set_progress(reset=True, stage="map", detail="sampling the course surfaces into map points")
    try:
        return _plan(req)
    finally:
        set_progress(active=False, finished=time.time())


def _plan(req):
    d = course_dir(req["id"])
    t, mp, pl = req["transform"], req["map"], req["planner"]
    runs = [n for n in os.listdir(d) if n.startswith("plan-")]
    run = "plan-%03d" % (len(runs) + 1)
    rd = os.path.join(d, run)
    os.makedirs(rd)

    v, f = map_vertices(req["id"], t)
    pts = cad_to_map.map_points(v[f], float(mp["spacing"]), bool(mp["addFloor"]))
    cad_to_map.write_map(os.path.join(rd, "map.bin"), pts)
    set_progress(detail="%d map points" % len(pts))

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
        set_progress(attempt=i + 1, tries=tries, stage="start", detail="starting gcopter_planner", leg=0, lines=[], routes=[])
        t_a = time.time()
        code, out = run_planner(rd, env, i + 1)
        log.append("== attempt %d/%d, exit %d\n%s" % (i + 1, tries, code, out))
        err = re.findall(r"ERROR: (.*)", out)
        with progress_lock:
            progress["attempts"] = progress.get("attempts", []) + [
                {"n": i + 1, "ok": code == 0, "s": round(time.time() - t_a, 2), "error": err[-1] if err else ""}]
        if code == 0:
            ok = True
            break
    log = "\n".join(log)
    with open(os.path.join(rd, "planner.log"), "w", encoding="utf-8") as fh:
        fh.write(log)

    res = {"ok": ok, "run": run, "log": log, "mapPoints": int(len(pts)), "routes": get_progress().get("routes", [])}
    if ok:
        set_progress(stage="clearance", detail="measuring distance to the course along the path")
        raw = open(os.path.join(rd, "path.bin"), "rb").read()
        rows = np.frombuffer(raw, "<f8", offset=32).reshape(-1, 12)
        p = ned_to_map(rows, settings["StartPos"], settings["StartHeadingDeg"])
        kor = settings["DilateRadius"]
        clear = path_clearance(p, v[f], pts, kor + float(mp["spacing"]), bool(mp["addFloor"]))
        res["path"] = {"t": rows[:, 0].tolist(), "pos": p.round(4).tolist(),
                       "speed": np.linalg.norm(rows[:, 4:7], axis=1).round(3).tolist(),
                       "clearance": clear.round(4).tolist()}
        res["keepOut"] = kor
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
            if u.path == "/api/progress":
                return self.send(200, get_progress())
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
            if u.path == "/api/inspect":
                return self.send(200, inspect(req))
            self.send(404, {"error": "not found"})
        except Exception as e:
            self.send(500, {"error": "%s: %s" % (type(e).__name__, e)})

    def log_message(self, fmt_, *args):
        if "/api/progress" not in self.path:      # the page polls it twice a second while planning
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
