#!/usr/bin/env bash
# Save one left raw frame, one left mono frame and the camera_info from a sim slot.
#   SIM_ID=1 bash grab_frames.sh     -> /tmp/frames_<N>/
source "$(dirname "$0")/sim/scripts/common.sh"
OUT=/tmp/frames_$SIM_ID; rm -rf $OUT; mkdir -p $OUT
cat > /tmp/grab_$SIM_ID.py <<'EOF'
import sys, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = Node("grab"); got = {}
def save(tag):
    def cb(m):
        if tag in got: return
        got[tag] = m
        ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(m.encoding, 1)
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch) if ch == 3 else np.frombuffer(bytes(m.data), np.uint8)[: m.height * m.step].reshape(m.height, m.step)[:, : m.width]
        print(tag, m.encoding, m.width, m.height, "mean", a.mean().round(1), "std", a.std().round(1), "stamp", m.header.stamp.sec, flush=True)
        hdr = f"P6\n{m.width} {m.height}\n255\n" if ch == 3 else f"P5\n{m.width} {m.height}\n255\n"
        open(f"/tmp/out/{tag}.p{'pm' if ch == 3 else 'gm'}", "wb").write(hdr.encode() + a.tobytes())
    return cb
n.create_subscription(Image, "/drone1/left/color/image_raw", save("raw_left"), 2)
n.create_subscription(Image, "/oak/left/image_mono", save("mono_left"), 2)
n.create_subscription(Image, "/oak/right/image_mono", save("mono_right"), 2)
n.create_subscription(CameraInfo, "/drone1/left/color/camera_info", lambda m: got.setdefault("info", m), 2)
import time; t0 = time.time()
while time.time() - t0 < 10 and len(got) < 4:
    rclpy.spin_once(n, timeout_sec=0.1)
i = got.get("info")
print("camera_info:", i and (i.width, i.height, list(i.k), i.distortion_model, list(i.d), list(i.p), i.header.frame_id))
EOF
docker exec "$CONTAINER" mkdir -p /tmp/out
docker cp /tmp/grab_$SIM_ID.py "$CONTAINER":/tmp/grab.py >/dev/null
docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && python3 /tmp/grab.py"
docker cp "$CONTAINER":/tmp/out/. $OUT/ >/dev/null
ls -la $OUT
