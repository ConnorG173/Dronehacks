"""Rewrite a possibly-unfinalized sim recording into a finalized, indexed MCAP on ONE clock (sim time).
Old recordings: log times were wall clock, and the viz course/checkpoints/trails plus the world->course TF were
wall-stamped while TF/drone/VIO were sim-stamped. Wall times are mapped to sim time with the offset measured on /tf."""
import sys
from mcap.stream_reader import StreamReader
from mcap.records import Schema, Channel, Message
from mcap.writer import Writer, CompressionType
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
src, dst = sys.argv[1], sys.argv[2]
WALL = 10**18          # ns; anything above is wall clock
FIX_SCENE = {"/viz/course", "/viz/checkpoints", "/viz/trails", "/viz/drone"}
recs = []
with open(src, "rb") as f:
    try:
        for r in StreamReader(f, skip_magic=False).records:
            if isinstance(r, (Schema, Channel, Message)): recs.append(r)
    except Exception as e:
        print(f"  (truncated tail: {type(e).__name__})")
chan = {r.id: r for r in recs if isinstance(r, Channel)}
sch = {r.id: r for r in recs if isinstance(r, Schema)}
# offset wall->sim from the first /tf message (header = sim time)
offset = 0
TF = get_message("tf2_msgs/msg/TFMessage")
for r in recs:
    if isinstance(r, Message) and chan[r.channel_id].topic == "/tf" and r.log_time > WALL:
        m = deserialize_message(r.data, TF); st = m.transforms[0].header.stamp
        offset = r.log_time - (st.sec * 10**9 + st.nanosec); break
def to_sim(ns): return ns - offset if ns > WALL else ns
def fix_stamp(t):
    ns = t.sec * 10**9 + t.nanosec
    if ns > WALL: ns = to_sim(ns); t.sec, t.nanosec = ns // 10**9, ns % 10**9
types = {}
smap, cmap, n, fixed = {}, {}, 0, 0
with open(dst, "wb") as o:
    w = Writer(o, compression=CompressionType.ZSTD); w.start(profile="ros2", library="fix_mcap")
    for r in recs:
        if isinstance(r, Schema) and r.id not in smap:
            smap[r.id] = w.register_schema(r.name, r.encoding, r.data)
        elif isinstance(r, Channel) and r.id not in cmap:
            cmap[r.id] = w.register_channel(r.topic, r.message_encoding, smap.get(r.schema_id, 0), dict(r.metadata))
        elif isinstance(r, Message) and r.channel_id in cmap:
            topic = chan[r.channel_id].topic; data = r.data
            if offset and (topic in FIX_SCENE or topic == "/tf_static"):
                tname = sch[chan[r.channel_id].schema_id].name
                T = types.setdefault(tname, get_message(tname)); m = deserialize_message(data, T)
                if topic == "/tf_static":
                    for tr in m.transforms: fix_stamp(tr.header.stamp)
                else:
                    for e in m.entities: fix_stamp(e.timestamp)
                data = serialize_message(m); fixed += 1
            w.add_message(cmap[r.channel_id], to_sim(r.log_time), data, to_sim(r.publish_time), r.sequence); n += 1
    w.finish()
print(f"{dst}: {n} messages, offset {offset/1e9:.1f} s, restamped {fixed}")
