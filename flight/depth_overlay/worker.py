#!/usr/bin/env python3
"""Passive Depth Anything display worker. Never publishes flight commands."""
import os
for _key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
from collections import deque
import fcntl
import json
from pathlib import Path
import sys
import threading
import time

import cv2
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus, VehicleLandDetected
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent.parent/'src'/'localizer'))
from localizer.depth import Stereo, DepthAnything, ScaleFit, img_to_np
from localizer.bev import quat_to_mat
from geometry import project_depth, nearest_sample, scaled_intrinsics, ScaleChooser


def stamp_ns(message):
    return message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec


def paired_images(left, right, last_stamp, now_ns):
    """Newest completed pair, including when the newest left is still unmatched."""
    for capture, image in sorted(left,key=lambda item:item[0],reverse=True):
        if capture<=last_stamp or not -50_000_000<=now_ns-capture<=350_000_000:
            continue
        mate=nearest_sample(right,capture,8_000_000)
        if mate is not None:
            return capture,image,mate
    return None


class OverlayWorker(Node):
    def __init__(self, args):
        super().__init__('depth_anything_overlay')
        self.args = args
        self.lock = threading.Lock()
        self.images = {'left': deque(maxlen=3), 'right': deque(maxlen=3)}
        self.infos = {}
        self.positions, self.attitudes = deque(maxlen=200), deque(maxlen=300)
        self.tracker = self.status = self.land = None
        self.stereo = self.camera = self.model = None
        self.scale_chooser = ScaleChooser()   # per-frame ML depth scale from this frame's stereo
        # EGO-Planner mode: metric depth + camera pose with the same stamp (published only if subscribed)
        self.ego_depth_pub = self.create_publisher(Image, '/ego/depth', 2)
        self.ego_pose_pub = self.create_publisher(PoseStamped, '/ego/camera_pose', 2)
        self.ego_K = None
        self.torch_configured = False
        self.last_stamp = 0
        self.frames = 0
        self.last_report = 0.0
        self.publisher = self.create_publisher(String, '/claude/depth/overlay', 1)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in ('left', 'right'):
            self.create_subscription(Image, '/oak/'+side+'/image_mono',
                lambda m,s=side: self.image(s,m), image_qos)
            self.create_subscription(CameraInfo, '/oak/'+side+'/camera_info',
                lambda m,s=side: self.info(s,m), qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, '/claude/usb/out/vehicle_local_position',
                                 self.position, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/claude/usb/out/vehicle_attitude',
                                 self.attitude, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/claude/usb/out/vehicle_status_v1',
                                 self.vehicle, qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected, '/claude/usb/out/vehicle_land_detected',
                                 self.landed, qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                 self.tracking, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        cv2.setNumThreads(1)

    def image(self, side, message):
        with self.lock:
            self.images[side].append((stamp_ns(message), message))

    def info(self, side, message):
        with self.lock:
            self.infos[side] = message

    def position(self, message):
        if message.xy_valid and message.z_valid:
            # Adapter timestamp is host receipt time; timestamp_sample is FC boot time.
            with self.lock:
                self.positions.append((int(message.timestamp)*1000,
                                       (float(message.x),float(message.y),float(message.z))))

    def attitude(self, message):
        with self.lock:
            self.attitudes.append((int(message.timestamp)*1000, tuple(float(v) for v in message.q)))

    def tracking(self, message):
        with self.lock:
            self.tracker = (time.monotonic(), int(message.vo_state))

    def vehicle(self, message):
        with self.lock:
            self.status = (time.monotonic(), message.arming_state == 2)

    def landed(self, message):
        with self.lock:
            self.land = (time.monotonic(), bool(message.landed))

    def transform(self, target, source):
        tr = self.tf.lookup_transform(target, source, Time()).transform
        q,t = tr.rotation,tr.translation
        return quat_to_mat(q.x,q.y,q.z,q.w), np.array([t.x,t.y,t.z])

    def write_view(self, rect, depth_m, label):
        """Small left-camera + colour depth JPEG for the dashboard (display only)."""
        try:
            w, h = self.model.w, self.model.h
            left = cv2.resize(rect, (w, h))
            if left.ndim == 2:
                left = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
            if depth_m is None:
                colour = np.full_like(left, 40)
            else:
                d = np.nan_to_num(np.clip(depth_m, .3, 4.), nan=4.)
                colour = cv2.applyColorMap((255*(4.-d)/3.7).astype(np.uint8), cv2.COLORMAP_TURBO)
                colour[~np.isfinite(depth_m)] = (30, 30, 30)
            image = np.hstack([left, colour])
            cv2.putText(image, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
            ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                temp = ROOT/'view.tmp'
                temp.write_bytes(jpg.tobytes())
                temp.replace(ROOT/'view.jpg')
        except Exception as exc:
            self.get_logger().warn(f'depth view not written: {exc}', throttle_duration_sec=10.)

    def publish_for_ego(self, depth_m, K, geometry, capture_ns):
        """/ego/depth (32FC1 metres, 0 = beyond range) and /ego/camera_pose (optical frame in ENU), same stamp."""
        try:
            if K != self.ego_K:                       # intrinsics for EGO's launch, at the model resolution
                self.ego_K = K
                h, w = depth_m.shape
                (ROOT/'ego_camera.json').write_text(json.dumps(dict(fx=K[0], fy=K[1], cx=K[2], cy=K[3],
                                                                    width=int(w), height=int(h))))
            if self.ego_depth_pub.get_subscription_count() == 0 or 'R_enu_cam' not in geometry:
                return
            stamp = Time(nanoseconds=int(capture_ns)).to_msg()
            img = Image()
            img.header.stamp, img.header.frame_id = stamp, 'ego_camera'
            d = np.nan_to_num(np.asarray(depth_m, dtype=np.float32), nan=0., posinf=0., neginf=0.)
            img.height, img.width = d.shape
            img.encoding, img.is_bigendian, img.step = '32FC1', 0, int(d.shape[1]*4)
            img.data = d.tobytes()
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, 'world'
            p = geometry['camera_enu']
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in p)
            R = np.asarray(geometry['R_enu_cam'], dtype=float)
            w_ = math.sqrt(max(0., 1.+R[0, 0]+R[1, 1]+R[2, 2]))/2.
            if w_ > 1e-6:
                q = (w_, (R[2, 1]-R[1, 2])/(4*w_), (R[0, 2]-R[2, 0])/(4*w_), (R[1, 0]-R[0, 1])/(4*w_))
            else:                                     # 180-degree case
                i = int(np.argmax(np.diag(R))); j, k = (i+1) % 3, (i+2) % 3
                s_ = math.sqrt(max(0., 1.+R[i, i]-R[j, j]-R[k, k]))*2
                v = [0., 0., 0.]; v[i] = s_/4; v[j] = (R[j, i]+R[i, j])/s_; v[k] = (R[k, i]+R[i, k])/s_
                q = ((R[k, j]-R[j, k])/s_, *v)
            pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = q
            self.ego_depth_pub.publish(img)
            self.ego_pose_pub.publish(pose)
        except Exception as exc:
            self.get_logger().warn(f'EGO depth not published: {exc}', throttle_duration_sec=10.)

    def emit(self, status, reason='', *, capture_ns=None, geometry=None, **metrics):
        result = dict(source='Depth Anything V2', frame='px4_local_enu', status=status,
                      reason=reason, stamp_ns=capture_ns, produced_wall_ns=time.time_ns(),
                      cell_m=.08, points=[], camera_xy=None, view_polygon=[],
                      frames=self.frames, **metrics)
        if geometry is not None:
            result.update(geometry)
        self.publisher.publish(String(data=json.dumps(result,allow_nan=False)))
        temp = ROOT/'status.tmp'
        temp.write_text(json.dumps(result,allow_nan=False)+'\n')
        temp.replace(ROOT/'status.json')
        if time.monotonic()-self.last_report > 10:
            self.get_logger().info(f'{status}: {reason or str(len(result["points"]))+" surface cells"}; frames={self.frames}')
            self.last_report = time.monotonic()

    def step(self):
        with self.lock:
            left, right = list(self.images['left']), list(self.images['right'])
            positions, attitudes = list(self.positions), list(self.attitudes)
            infos = dict(self.infos)
            tracker,status,land = self.tracker,self.status,self.land
        now = time.monotonic()
        if tracker is None or now-tracker[0]>.5 or tracker[1]!=1:
            return self.emit('waiting','Waiting for fresh camera tracking')
        if not left or not right or len(infos)!=2:
            return self.emit('waiting','Waiting for stereo images and calibration')
        pair=paired_images(left,right,self.last_stamp,time.time_ns())
        if pair is None:
            return self.emit('waiting','Waiting for a fresh synchronized image pair')
        capture,l,r=pair
        position = nearest_sample(positions,capture)
        attitude = nearest_sample(attitudes,capture)
        if position is None or attitude is None:
            return self.emit('waiting','Waiting for position aligned with the image')
        self.last_stamp = capture
        a,b = infos['left'],infos['right']
        if (l.width,l.height)!=(a.width,a.height) or (r.width,r.height)!=(b.width,b.height):
            return self.emit('waiting','Camera info does not match image size yet')
        if self.stereo is not None and self.stereo_src!=(a.width,a.height):
            self.get_logger().warn(f'Camera size changed {self.stereo_src} -> {(a.width,a.height)}; recalibrating')
            self.stereo = None
        if self.stereo is None:
            if (a.width,a.height)!=(b.width,b.height):
                return self.emit('error','Stereo image sizes differ')
            try:
                lr = self.transform(a.header.frame_id,b.header.frame_id)
                bc = self.transform('base_link',a.header.frame_id)
            except Exception:
                return self.emit('waiting','Waiting for camera transforms')
            # ponytail: always rectify to 512 px wide, the size the depth bench was measured at
            self.stereo = Stereo(a.k,a.d,b.k,b.d,(a.width,a.height),*lr,scale=512/a.width)
            self.stereo_src = (a.width,a.height)
            self.camera = (bc[0] @ self.stereo.R1.T, bc[1])
            self.get_logger().info(f'Calibrated stereo: {self.stereo.size}, baseline={self.stereo.baseline:.4f}m')
        if self.model is None:
            # GPU context setup/warm-up is deferred until the vehicle is grounded.
            if (status is None or land is None or now-status[0]>2 or now-land[0]>1.5
                    or status[1] or not land[1]):
                return self.emit('waiting','Model startup waits for disarmed and landed state')
            self.emit('waiting','Starting Depth Anything')
            import torch
            if not self.torch_configured:
                torch.set_num_threads(1)
                torch.set_num_interop_threads(1)
                self.torch_configured = True
            self.model = DepthAnything(self.args.engine)
            self.model(np.zeros((320,512),dtype=np.uint8))
            return  # Discard the image captured before model startup.
        begin = time.perf_counter()
        rect, stereo_depth = self.stereo.depth(img_to_np(l),img_to_np(r))
        t_stereo = time.perf_counter()
        relative = self.model(rect)
        t_model = time.perf_counter()
        stereo_small = cv2.resize(stereo_depth,(self.model.w,self.model.h),interpolation=cv2.INTER_NEAREST)
        if (ROOT/'dump_scale').exists():     # debug: touch dump_scale to save this frame's scale inputs
            try:
                (ROOT/'dump_scale').unlink()
                np.savez_compressed(ROOT/'scale_dump.npz', rel=relative, stereo=stereo_small, rect=rect)
            except OSError:
                pass
        # Each image still needs its own valid metric fit. Several RANSAC fits (8 x 80 tries) are the
        # candidates; the chooser scores them, and last frame's scale, on THIS frame's stereo and keeps
        # last frame's while it still fits, else the best, preferring more contrast on a tie.
        fits = []
        for seed in range(8):
            f = ScaleFit(seed=seed)
            if f.update(relative, stereo_small):
                fits.append(f)
        q = np.nanpercentile(relative, [10, 50, 90])
        chosen = self.scale_chooser.choose([(f.s, f.t) for f in fits], relative, stereo_small, q) if fits else (None, 'none', 0.)
        if chosen[0] is None:
            valid = int((np.isfinite(stereo_small) & (stereo_small > .3) & (stereo_small < 3.5)).sum())
            why = (f'No usable scale among {len(fits)} fits' if fits else
                   f'Not enough stereo detail to scale ML depth ({valid} stereo pixels)')
            if (ROOT/'dump_unscaled').exists():   # debug: touch dump_unscaled to save the next failing frame
                try:
                    (ROOT/'dump_unscaled').unlink()
                    np.savez_compressed(ROOT/'unscaled_dump.npz', rel=relative, stereo=stereo_small, rect=rect,
                                        candidates=np.array([(f_.s, f_.t, f_.n_in) for f_ in fits]).reshape(-1, 3))
                except OSError:
                    pass
            self.write_view(rect, None, 'UNSCALED - ' + why)
            return self.emit('unscaled', why, capture_ns=capture, stereo_valid=valid, n_fits=len(fits))
        (fit, (st, gate_state, support)) = fits[0], chosen
        fit.s, fit.t = st
        candidates = sorted({(round(float(f_.s), 2), round(float(f_.t), 2), int(f_.n_in)) for f_ in fits},
                            key=lambda k: -k[2])
        depth = fit.metric(relative,zmax=4.0)
        self.write_view(rect, depth, f'depth 0.3-4 m, near = red (scale {gate_state})')
        K = scaled_intrinsics(self.stereo.K,self.stereo.size,(self.model.w,self.model.h))
        geometry = project_depth(depth,K,*self.camera,position,attitude)
        self.publish_for_ego(depth, K, geometry, capture)
        elapsed = time.perf_counter()-begin
        if (time.time_ns()-capture)/1e9>1.0:
            return self.emit('waiting','Depth frame expired during processing',capture_ns=capture)
        self.frames += 1
        self.emit('live',capture_ns=capture,geometry=geometry,
                  ms_stereo=round((t_stereo-begin)*1000,1),
                  ms_model=round((t_model-t_stereo)*1000,1), ms_total=round(elapsed*1000,1),
                  scale_inliers=int(fit.n_in), scale_gate=gate_state, scale_support=round(float(support), 3),
                  scale_s=round(float(fit.s), 5), scale_t=round(float(fit.t), 5),
                  scale_candidates=[list(k) for k in candidates],
                  pose_stamp_delta_ms=round(min(abs(t-capture) for t,_ in positions)/1e6,1),
                  depth_fill=round(float(np.isfinite(depth).mean()),3))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hz',type=float,default=2.0)
    parser.add_argument('--seconds',type=float,default=0,help='Optional bounded verification run')
    parser.add_argument('--engine',default=str(ROOT.parent/'models/da2s_252x406_fp16.engine'))
    args=parser.parse_args()
    if not 0<args.hz<=2:
        parser.error('display worker rate must be >0 and <=2Hz')
    guard=(ROOT/'worker.lock').open('w')
    try:
        fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Depth overlay worker already running')
    rclpy.init()
    node=OverlayWorker(args)
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
    thread.start()
    started=time.monotonic()
    try:
        while rclpy.ok() and (not args.seconds or time.monotonic()-started<args.seconds):
            t=time.monotonic()
            try:
                node.step()
            except Exception as exc:
                node.get_logger().error(str(exc))
                node.emit('error','Depth processing unavailable')
                time.sleep(5)
            time.sleep(max(.01,1/args.hz-(time.monotonic()-t)))
    except KeyboardInterrupt:
        pass
    finally:
        node.emit('waiting','Depth worker stopped')
        if rclpy.ok():rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        guard.close()


if __name__=='__main__':main()
