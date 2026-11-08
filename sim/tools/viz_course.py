#!/usr/bin/env python3
"""Foxglove 3D view of a sim slot: the course, checkpoints, the drone, and true vs VIO trails.

Run by sim/run_sim.sh (tmux window "viz") on the host, in the slot's ROS domain. Publishes, all in frame `course`:
  /viz/course        foxglove_msgs/SceneUpdate  every course part as a coloured 3D model (from the generated OBJ)
  /viz/checkpoints   foxglove_msgs/SceneUpdate  the planner config's checkpoint boxes + the goal
  /viz/drone         foxglove_msgs/SceneUpdate  a Sky Beta sized body + prop discs, in frame `drone_gt`
  /viz/trails        foxglove_msgs/SceneUpdate  orange = ground truth, cyan = cuVSLAM mapped into the course frame
  TF course -> drone_gt                         from /sim/ground_truth/odom
Open with Foxglove: ws://localhost:876<slot+5> and import docs/foxglove_sim_layout.json.
"""
import math
import os
import struct
import sys

import numpy as np
import rclpy
from foxglove_msgs.msg import (Color, CubePrimitive, CylinderPrimitive, LinePrimitive, SceneEntity, SceneUpdate,
                               TriangleListPrimitive)
from geometry_msgs.msg import Point, Pose, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

REPO = os.environ.get("REPO", os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
OBJ = os.path.join(REPO, "sim/gz/models/dronehacks_course/meshes/course.obj")
COURSE = os.path.join(REPO, os.environ.get("COURSE", "planner/config/course_v3_gateb.yaml"))
MOTORS_FLU = [(0.08, -0.16), (-0.15, 0.14), (0.08, 0.16), (-0.15, -0.14)]   # Sky Beta CA_ROTOR, FRD -> FLU
PALETTE = [(0.92, 0.55, 0.35), (0.40, 0.65, 0.90), (0.55, 0.80, 0.45), (0.85, 0.75, 0.40),
           (0.70, 0.55, 0.85), (0.45, 0.80, 0.80), (0.90, 0.60, 0.70), (0.65, 0.65, 0.65)]


def col(r, g, b, a=1.0):
    c = Color(); c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a); return c


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    p = Pose(); p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2); return p


def load_obj_groups(path):
    """The generator writes each OBJ face with its own 3 vertices, so faces are consecutive vertex triples."""
    groups, verts, cur = {}, [], None
    with open(path) as f:
        for line in f:
            if line.startswith("o "):
                cur = line.split()[1]; groups.setdefault(cur, [])
            elif line.startswith("v "):
                verts.append([float(v) for v in line.split()[1:4]])
            elif line.startswith("f ") and cur is not None:
                idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:4]]
                groups[cur].append(idx)
    V = np.asarray(verts, dtype=np.float32)
    return {name: V[np.asarray(faces)] for name, faces in groups.items() if faces}


def stl_bytes(tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    return b"\0" * 80 + struct.pack("<I", len(tris)) + rec.tobytes()


def read_course_config(path):
    start, heading, cps, goal = (0.0, 0.0, 0.0), 0.0, [], None
    if not os.path.exists(path):   # e.g. a checkout without the Gate B config: fall back to the window route
        path = os.path.join(REPO, "planner/config/course_v3.yaml")
    for line in open(path):
        line = line.split("#")[0].strip()
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        if k not in ("StartPos", "StartHeadingDeg", "Checkpoint", "Goal"):
            continue
        nums = [float(t) for t in v.replace("[", " ").replace("]", " ").replace(",", " ").split()]
        if k == "StartPos": start = tuple(nums[:3])
        elif k == "StartHeadingDeg": heading = math.radians(nums[0])
        elif k == "Checkpoint": cps.append(nums[:6])
        elif k == "Goal": goal = nums[:3]
    return start, heading, cps, goal


class Viz(Node):
    def __init__(self):
        super().__init__("viz_course")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_course = self.create_publisher(SceneUpdate, "/viz/course", latched)
        self.pub_cps = self.create_publisher(SceneUpdate, "/viz/checkpoints", latched)
        self.pub_drone = self.create_publisher(SceneUpdate, "/viz/drone", 10)
        self.pub_trails = self.create_publisher(SceneUpdate, "/viz/trails", 10)
        self.tf = TransformBroadcaster(self)
        # Always publish the course frame (static world -> course), so Foxglove can draw the course even when
        # ground truth is missing (the drone then follows the VIO estimate instead).
        self.static_tf = StaticTransformBroadcaster(self)
        st = TransformStamped(); st.header.frame_id = "world"; st.child_frame_id = "course"; st.transform.rotation.w = 1.0
        st.header.stamp = self.get_clock().now().to_msg(); self.static_tf.sendTransform(st)
        self.last_gt = 0.0
        self.start, self.heading, self.cps, self.goal = read_course_config(COURSE)
        self.course_msg = self.build_course()
        self.cps_msg = self.build_checkpoints()
        self.gt, self.vio = [], []
        self.create_subscription(Odometry, "/sim/ground_truth/odom", self.on_gt, 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.on_vio, 10)
        self.create_timer(5.0, self.republish_static)
        self.create_timer(0.2, self.publish_trails)
        self.republish_static()
        self.get_logger().info(f"course parts: {len(self.course_msg.entities)}, checkpoints: {len(self.cps)} ({COURSE})")

    def stamp(self):
        return self.get_clock().now().to_msg()

    def build_course(self):
        # Native TriangleListPrimitive with lighting baked into per-vertex colours (renders everywhere; embedded STL
        # models did not show up in Foxglove).
        up = SceneUpdate()
        light = np.array([0.35, 0.25, 0.9]); light /= np.linalg.norm(light)
        for i, (name, tris) in enumerate(sorted(load_obj_groups(OBJ).items())):
            base = np.array((0.30, 0.30, 0.34) if name == "floor" else PALETTE[i % len(PALETTE)])
            n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
            n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
            shade = 0.45 + 0.55 * np.abs(n @ light)          # two-sided
            e = SceneEntity(); e.frame_id = "course"; e.id = f"course_{name}"; e.frame_locked = True
            t = TriangleListPrimitive(); t.pose = pose()
            t.points = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for tri in tris for v in tri]
            t.colors = [col(*(base * k)) for k in shade for _ in range(3)]
            e.triangles.append(t); up.entities.append(e)
        return up

    def build_checkpoints(self):
        up = SceneUpdate(); e = SceneEntity(); e.frame_id = "course"; e.id = "checkpoints"; e.frame_locked = True
        for c in self.cps:
            cube = CubePrimitive(); cube.pose = pose(c[0], c[1], c[2])
            cube.size = Vector3(x=2 * c[3], y=2 * c[4], z=2 * c[5]); cube.color = col(0.2, 0.95, 0.3, 0.35)
            e.cubes.append(cube)
        if self.goal:
            pad = CylinderPrimitive(); pad.pose = pose(self.goal[0], self.goal[1], 0.01)
            pad.size = Vector3(x=1.52, y=1.52, z=0.02); pad.bottom_scale = pad.top_scale = 1.0
            pad.color = col(1.0, 0.85, 0.1, 0.6); e.cylinders.append(pad)
        up.entities.append(e)
        return up

    def republish_static(self):
        for msg in (self.course_msg, self.cps_msg):
            for e in msg.entities:
                e.timestamp = self.stamp()
        self.pub_course.publish(self.course_msg)
        self.pub_cps.publish(self.cps_msg)

    def on_gt(self, m):
        self.last_gt = self.get_clock().now().nanoseconds * 1e-9
        self.draw_drone(m.header.stamp, m.pose.pose.position, m.pose.pose.orientation, gt=True)

    def draw_drone(self, stamp, p, q, gt):
        t = TransformStamped(); t.header.stamp = stamp; t.header.frame_id = "course"; t.child_frame_id = "drone_gt"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.tf.sendTransform(t)
        if gt:
            if self.gt and math.dist(self.gt[-1], (p.x, p.y, p.z)) > 1.0:   # respawn: start a new trail
                self.gt.clear(); self.vio.clear()
            if not self.gt or math.dist(self.gt[-1], (p.x, p.y, p.z)) > 0.02:
                self.gt.append((p.x, p.y, p.z)); self.gt = self.gt[-4000:]
        up = SceneUpdate(); e = SceneEntity(); e.frame_id = "drone_gt"; e.id = "drone"; e.timestamp = stamp
        body = CubePrimitive(); body.pose = pose(z=0.05); body.size = Vector3(x=0.22, y=0.16, z=0.07)
        body.color = col(0.1, 0.1, 0.12); e.cubes.append(body)
        nose = CubePrimitive(); nose.pose = pose(0.15, 0.0, 0.05); nose.size = Vector3(x=0.08, y=0.03, z=0.03)
        nose.color = col(0.95, 0.15, 0.15); e.cubes.append(nose)
        for x, y in MOTORS_FLU:
            d = CylinderPrimitive(); d.pose = pose(x, y, 0.09); d.size = Vector3(x=0.178, y=0.178, z=0.01)
            d.bottom_scale = d.top_scale = 1.0; d.color = col(0.3, 0.75, 1.0, 0.5); e.cylinders.append(d)
        up.entities.append(e); self.pub_drone.publish(up)

    def on_vio(self, m):
        # cuVSLAM odom starts at the launch pose: map it into the course frame with the config's start pose.
        p = m.pose.pose.position; c, s = math.cos(self.heading), math.sin(self.heading)
        pt = (self.start[0] + c * p.x - s * p.y, self.start[1] + s * p.x + c * p.y, self.start[2] + p.z)
        if not self.vio or math.dist(self.vio[-1], pt) > 0.02:
            self.vio.append(pt); self.vio = self.vio[-4000:]
        if self.get_clock().now().nanoseconds * 1e-9 - self.last_gt > 1.0:   # no ground truth: show the VIO pose
            q = m.pose.pose.orientation; h = self.heading
            qz, qw = math.sin(h / 2), math.cos(h / 2)   # rotate VIO orientation by the start heading
            q2 = type(q)(); q2.w = qw * q.w - qz * q.z; q2.x = qw * q.x - qz * q.y; q2.y = qw * q.y + qz * q.x; q2.z = qw * q.z + qz * q.w
            self.draw_drone(m.header.stamp, Point(x=pt[0], y=pt[1], z=pt[2]), q2, gt=False)

    def publish_trails(self):
        up = SceneUpdate(); e = SceneEntity(); e.frame_id = "course"; e.id = "trails"; e.timestamp = self.stamp()
        for pts, color in ((self.gt, col(1.0, 0.55, 0.1)), (self.vio, col(0.1, 0.9, 1.0))):
            if len(pts) < 2:
                continue
            ln = LinePrimitive(); ln.type = LinePrimitive.LINE_STRIP; ln.pose = pose(); ln.thickness = 0.03
            ln.scale_invariant = False; ln.color = color; ln.points = [Point(x=a, y=b, z=z) for a, b, z in pts]
            e.lines.append(ln)
        up.entities.append(e); self.pub_trails.publish(up)


def main():
    if not os.path.exists(OBJ):
        sys.exit(f"no course mesh at {OBJ}: start the sim once (run_sim.sh generates it)")
    rclpy.init(); rclpy.spin(Viz())


if __name__ == "__main__":
    main()
