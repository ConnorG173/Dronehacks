#!/usr/bin/env python3
"""Course STEP -> textured Gazebo model (OBJ + MTL + PNG textures) in the planner's map frame.

    python3 sim/tools/step_to_gz.py dronehacks-course-v3.step --out sim/gz/models/dronehacks_course

Same transform as planner/tools/cad_to_map.py (defaults match planner/config/course_v3.yaml:
CAD mm, z up, no yaw/offset), so Gazebo world == map frame and the planner's coordinates are
directly the Gazebo coordinates.

cuVSLAM needs texture: plain CAD surfaces are uniform grey and would make VIO fail for the wrong
reason. Every solid gets one of several procedural, high-contrast textures (triplanar UVs, tiled
every --tile metres). Real course surfaces are unknown; this is a stand-in, not a photo match.
A textured floor slab is added (the CAD floor, if any, is kept too).

Needs: cadquery (STEP import), numpy, trimesh, pillow.
"""
import argparse
import math
import os

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFilter


def load_step_solids(path, tol_mm):
    import cadquery as cq
    shape = cq.importers.importStep(path)
    out = []
    for solid in shape.solids().vals():
        verts, tris = solid.tessellate(tol_mm, 0.3)
        if not tris:
            continue
        v = np.array([(p.x, p.y, p.z) for p in verts], dtype=float)
        f = np.array(tris, dtype=np.int64)
        out.append(trimesh.Trimesh(v, f, process=True))
    return out


def transform(points, scale, up, yaw_deg, translate):
    p = points * scale
    if up == "y":
        p = np.stack([p[:, 0], -p[:, 2], p[:, 1]], axis=1)
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    p = np.stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1], p[:, 2]], axis=1)
    return p + np.asarray(translate, dtype=float)


def make_texture(seed, size=2048):
    """High-contrast blotches + rectangles + fine noise: lots of corners for feature tracking."""
    rng = np.random.default_rng(seed)
    hue = rng.uniform(0, 1)
    base = np.array([0.55 + 0.35 * math.sin(2 * math.pi * (hue + k / 3)) for k in range(3)]) * 255
    img = Image.new("RGB", (size, size), tuple(int(x) for x in base * 0.8))
    d = ImageDraw.Draw(img)
    for _ in range(3500):   # dense: one texture spans --tile metres (2.5 m), so keep features ~1-10 cm
        x, y = rng.integers(0, size, 2)
        w, h = rng.integers(6, 48, 2)
        shade = rng.uniform(0.15, 1.0)
        col = tuple(int(min(255, c * shade + rng.uniform(-30, 30))) for c in base)
        if rng.random() < 0.5:
            d.rectangle([x, y, x + w, y + h], fill=col)
        else:
            d.ellipse([x, y, x + w, y + h], fill=col)
    img = img.filter(ImageFilter.GaussianBlur(0.8))
    noise = rng.normal(0, 14, (size, size, 1))
    arr = np.clip(np.asarray(img, dtype=float) + noise, 0, 255).astype(np.uint8)
    return Image.fromarray(arr)


def triplanar_uv(tri_pts, tile):
    """Per-face UVs from the dominant normal axis; tri_pts (3, 3)."""
    n = np.cross(tri_pts[1] - tri_pts[0], tri_pts[2] - tri_pts[0])
    axis = int(np.argmax(np.abs(n)))
    keep = [a for a in range(3) if a != axis]
    return tri_pts[:, keep] / tile


def floor_slab(bounds_xy, z_top=0.0, thickness=0.02):
    (xmin, ymin), (xmax, ymax) = bounds_xy
    box = trimesh.creation.box(extents=[xmax - xmin, ymax - ymin, thickness])
    box.apply_translation([(xmin + xmax) / 2, (ymin + ymax) / 2, z_top - thickness / 2])
    return box


def write_obj(meshes_by_mat, out_dir, tile):
    mesh_dir = os.path.join(out_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    obj_lines, mtl_lines = ["mtllib course.mtl"], []
    v_off = 1
    n_off = 1
    rng = np.random.default_rng(12345)
    for mat, meshes in meshes_by_mat.items():
        uv_off = rng.uniform(0, 1, 2)   # per-material pattern offset, so adjacent parts never line up
        tex = f"{mat}.png"
        mtl_lines += [f"newmtl {mat}", "Ka 0 0 0", "Kd 1 1 1", "Ks 0.05 0.05 0.05", "Ns 10",
                      "d 1", "illum 2", f"map_Kd {tex}", ""]
        obj_lines.append(f"o {mat}")
        obj_lines.append(f"usemtl {mat}")
        for m in meshes:
            for face in m.faces:
                pts = m.vertices[face]
                uv = triplanar_uv(pts, tile) + uv_off
                for p in pts:
                    obj_lines.append(f"v {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}")
                for t in uv:
                    obj_lines.append(f"vt {t[0]:.5f} {t[1]:.5f}")
                # Explicit face normal: without `vn` the renderer has no normals to light with, and the
                # whole course came out saturated white (no texture, no shading, nothing for VIO to track).
                nrm = np.cross(pts[1] - pts[0], pts[2] - pts[0])
                ln = np.linalg.norm(nrm)
                nrm = nrm / ln if ln > 0 else np.array([0.0, 0.0, 1.0])
                obj_lines.append(f"vn {nrm[0]:.5f} {nrm[1]:.5f} {nrm[2]:.5f}")
                a, b, c = v_off, v_off + 1, v_off + 2
                k = n_off
                obj_lines.append(f"f {a}/{a}/{k} {b}/{b}/{k} {c}/{c}/{k}")
                v_off += 3
                n_off += 1
    with open(os.path.join(mesh_dir, "course.obj"), "w") as f:
        f.write("\n".join(obj_lines) + "\n")
    with open(os.path.join(mesh_dir, "course.mtl"), "w") as f:
        f.write("\n".join(mtl_lines) + "\n")
    return mesh_dir


MODEL_SDF_HEAD = """<?xml version="1.0"?>
<!-- Generated by sim/tools/step_to_gz.py from {src}. Do not edit by hand.
     One visual per material: each OBJ object is a submesh with its own PBR albedo texture. Relying on the
     OBJ's MTL rendered every surface flat white in gz ogre2, which leaves cuVSLAM nothing to track. -->
<sdf version="1.9">
  <model name="dronehacks_course">
    <static>true</static>
    <link name="link">
"""
MODEL_SDF_VISUAL = """      <visual name="visual_{mat}">
        <geometry><mesh><uri>model://dronehacks_course/meshes/course.obj</uri>
          <submesh><name>{mat}</name><center>false</center></submesh></mesh></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>
          <pbr><metal><albedo_map>model://dronehacks_course/meshes/{mat}.png</albedo_map><roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
"""
MODEL_SDF_TAIL = """      <collision name="collision">
        <geometry><mesh><uri>model://dronehacks_course/meshes/collision.stl</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model><name>dronehacks_course</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>DroneHacks 2026 course v3, generated from the organizers' STEP.</description></model>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step")
    ap.add_argument("--out", required=True, help="model directory to write")
    ap.add_argument("--scale", type=float, default=0.001)
    ap.add_argument("--up", choices=["z", "y"], default="z")
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--translate", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    ap.add_argument("--tol-mm", type=float, default=2.0, help="tessellation tolerance in CAD units")
    ap.add_argument("--tile", type=float, default=2.5, help="texture repeat (m); large so a wall shows no repeats")
    ap.add_argument("--textures", type=int, default=0, help="distinct textures; 0 = one unique texture per solid")
    ap.add_argument("--floor-half", type=float, default=4.0, help="half-size of the added floor slab (m)")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()

    solids = load_step_solids(a.step, a.tol_mm)
    print(f"{len(solids)} solids")
    meshes_by_mat = {}
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for i, m in enumerate(solids):
        m.vertices = transform(np.asarray(m.vertices), a.scale, a.up, a.yaw_deg, a.translate)
        lo, hi = np.minimum(lo, m.vertices.min(0)), np.maximum(hi, m.vertices.max(0))
        # One unique texture per solid (default): with a few shared, tiled textures, walls in different places
        # looked identical, and cuVSLAM snapped between them (VIO jumps of 0.3-8 m at the same spot every flight).
        k = i % a.textures if a.textures > 0 else i
        meshes_by_mat.setdefault(f"tex{k}", []).append(m)
    print("course bounds (map frame, m):", np.round(lo, 3), np.round(hi, 3))
    floor_z = max(lo[2], 0.0) if lo[2] > -0.05 else 0.0
    meshes_by_mat["floor"] = [floor_slab(((-a.floor_half, -a.floor_half), (a.floor_half, a.floor_half)), floor_z - 0.001)]

    mesh_dir = write_obj(meshes_by_mat, a.out, a.tile)
    # Collision: one clean mesh (shared vertices, no degenerate/duplicate faces). ODE's trimesh asserts
    # on the per-face-vertex visual OBJ.
    col = trimesh.util.concatenate([m for ms in meshes_by_mat.values() for m in ms])
    col.merge_vertices()
    col.update_faces(col.nondegenerate_faces(height=1e-6))
    col.update_faces(col.unique_faces())
    col.remove_unreferenced_vertices()
    col.export(os.path.join(mesh_dir, "collision.stl"))
    for k, name in enumerate(meshes_by_mat):
        make_texture(a.seed * 100 + k).save(os.path.join(mesh_dir, f"{name}.png"))
    with open(os.path.join(a.out, "model.sdf"), "w") as f:
        f.write(MODEL_SDF_HEAD.format(src=os.path.basename(a.step)))
        for mat in meshes_by_mat:
            f.write(MODEL_SDF_VISUAL.format(mat=mat))
        f.write(MODEL_SDF_TAIL)
    with open(os.path.join(a.out, "model.config"), "w") as f:
        f.write(MODEL_CONFIG)
    n_tri = sum(len(m.faces) for ms in meshes_by_mat.values() for m in ms)
    print(f"wrote {a.out} ({n_tri} triangles, {len(meshes_by_mat)} materials)")


if __name__ == "__main__":
    main()
