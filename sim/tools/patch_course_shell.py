"""Patch recorded /viz/course messages: drop the cage roof and make the cage walls nearly transparent (same rule
as viz_course.shell_kind). Input must be a finalized MCAP (see fix_mcap.py). Needs ROS (rclpy) sourced.
    python3 patch_course_shell.py in.mcap out.mcap"""
import os, sys
import numpy as np
from mcap.reader import make_reader
from mcap.writer import Writer, CompressionType
from rclpy.serialization import deserialize_message, serialize_message
from foxglove_msgs.msg import SceneUpdate
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("REPO", os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from viz_course import shell_kind, SHELL_ALPHA


def stl_points(b):
    """Vertices of a binary STL (80-byte header, uint32 count, 50-byte facets)."""
    if len(b) < 84:
        return np.zeros((0, 3))
    n = int(np.frombuffer(b, np.uint32, 1, 80)[0])
    rec = np.frombuffer(b, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84)
    return rec["v"].reshape(-1, 3)


src, dst = sys.argv[1], sys.argv[2]
r = make_reader(open(src, "rb")); summ = r.get_summary()
with open(dst, "wb") as o:
    w = Writer(o, compression=CompressionType.ZSTD); w.start(profile="ros2", library="patch_course_shell")
    smap = {sid: w.register_schema(s.name, s.encoding, s.data) for sid, s in summ.schemas.items()}
    cmap = {cid: w.register_channel(c.topic, c.message_encoding, smap.get(c.schema_id, 0), dict(c.metadata))
            for cid, c in summ.channels.items()}
    patched = 0
    for sc, ch, m in r.iter_messages(log_time_order=True):
        data = m.data
        if ch.topic == "/viz/course":
            u = deserialize_message(data, SceneUpdate); keep = []
            for e in u.entities:
                pts = [(p.x, p.y, p.z) for t in e.triangles for p in t.points]
                for mdl in e.models:          # older recordings: embedded binary STL per part
                    pts += [tuple(x) for x in stl_points(bytes(mdl.data))]
                if pts:
                    v = np.array(pts); kind = shell_kind(v.min(0), v.max(0))
                    if kind == "roof":
                        continue
                    if kind == "wall":
                        for t in e.triangles:
                            for c in t.colors: c.a = SHELL_ALPHA
                        for mdl in e.models:
                            mdl.override_color = True; mdl.color.a = SHELL_ALPHA
                keep.append(e)
            u.entities = keep; data = serialize_message(u); patched += 1
        w.add_message(cmap[ch.id], m.log_time, data, m.publish_time, m.sequence)
    w.finish()
print(f"{dst}: patched {patched} /viz/course messages")
