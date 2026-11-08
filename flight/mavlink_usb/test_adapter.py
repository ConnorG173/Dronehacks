from collections import Counter
import math
import json
import os
from pathlib import Path
import select
import socket
import tempfile
import time
from types import SimpleNamespace as NS
import unittest

from pymavlink.dialects.v20 import common as mav
import numpy as np
from conversion import odometry_message, odometry_metadata, setpoint_message, SetpointPair
from serial_router import LatestQueue, SerialRouter, split_packets


def vision(now):
    return NS(timestamp=now, timestamp_sample=now-60_000, pose_frame=2, velocity_frame=3,
              position=[1.2,-3.4,.5], q=[math.sqrt(.5),0,0,math.sqrt(.5)],
              velocity=[.1,.2,-.3], angular_velocity=[.01,.02,.03],
              position_variance=[.01,.02,.03], orientation_variance=[.04,.05,.06],
              velocity_variance=[.07,.08,.09], reset_counter=7, quality=0)


def control(now, velocity=False):
    nan = float('nan')
    mode = NS(timestamp=now,position=not velocity,velocity=velocity,acceleration=False,
              attitude=False,body_rate=False)
    sp = NS(timestamp=now,position=[nan]*3 if velocity else [1.2,-3.4,.5],
            velocity=[.1,.2,-.3] if velocity else [nan]*3,acceleration=[nan]*3,
            yaw=nan,yawspeed=.2 if velocity else nan)
    return mode,sp


def wire_roundtrip(msg):
    sender=mav.MAVLink(None,srcSystem=12,srcComponent=197)
    return mav.MAVLink(None).parse_char(msg.pack(sender))


class ConversionTests(unittest.TestCase):
    def test_vision_wire_contract_preserves_px4_frames_and_sample_time(self):
        now=time.time_ns()//1000
        source=vision(now)
        out=wire_roundtrip(odometry_message(source,now))
        self.assertEqual((out.frame_id,out.child_frame_id),(20,12))
        self.assertEqual(out.time_usec,now-60_000)
        self.assertEqual(out.reset_counter,7)
        self.assertEqual(out.estimator_type,mav.MAV_ESTIMATOR_TYPE_VIO)
        for actual,expected in zip([out.x,out.y,out.z],source.position):
            self.assertAlmostEqual(actual,expected,places=5)
        for actual,expected in zip(out.q,source.q):
            self.assertAlmostEqual(actual,expected,places=6)
        for index,expected in zip([0,6,11,15,18,20],[.01,.02,.03,.04,.05,.06]):
            self.assertAlmostEqual(out.pose_covariance[index],expected,places=6)
        for index,expected in zip([0,6,11],[.07,.08,.09]):
            self.assertAlmostEqual(out.velocity_covariance[index],expected,places=6)

    def test_position_hold_has_no_unintended_yaw_velocity_or_thrust(self):
        now=time.time_ns()//1000
        mode,sp=control(now)
        out=wire_roundtrip(setpoint_message(sp,mode,now))
        self.assertEqual((out.target_system,out.target_component,out.coordinate_frame),(12,1,1))
        self.assertEqual(out.type_mask,3576)
        self.assertEqual(out.type_mask & 512,0)  # FORCE_SET never enabled
        self.assertAlmostEqual(out.y,-3.4,places=5)

    def test_velocity_has_no_position_command_and_preserves_yaw_rate_sign(self):
        now=time.time_ns()//1000
        mode,sp=control(now,True)
        out=wire_roundtrip(setpoint_message(sp,mode,now))
        self.assertEqual(out.type_mask,1479)
        self.assertAlmostEqual(out.vy,.2,places=6)
        self.assertAlmostEqual(out.yaw_rate,.2,places=6)

    def test_stale_vision_and_invalid_quaternion_are_rejected(self):
        now=time.time_ns()//1000
        sample=vision(now)
        sample.timestamp_sample=now-400_000
        with self.assertRaises(ValueError): odometry_message(sample,now)
        sample=vision(now);sample.q=[0,0,0,0]
        with self.assertRaises(ValueError): odometry_message(sample,now)

    def test_control_pairs_out_of_order_without_replaying_old_ticks(self):
        now=time.time_ns()//1000
        mode,sp=control(now)
        pair=SetpointPair()
        self.assertIsNone(pair.update('setpoint',sp,now))
        self.assertIsNotNone(pair.update('mode',mode,now))
        self.assertIsNone(pair.update('mode',mode,now))
        self.assertIsNone(pair.update('setpoint',sp,now))
        mode.timestamp=now+100_000
        self.assertIsNone(pair.update('mode',mode,now+100_000))
        sp.timestamp=now+100_000
        self.assertIsNotNone(pair.update('setpoint',sp,now+100_000))

    def test_stale_or_mismatched_control_cannot_keep_offboard_alive(self):
        now=time.time_ns()//1000
        mode,sp=control(now-300_000)
        with self.assertRaises(ValueError): setpoint_message(sp,mode,now)
        mode,sp=control(now);mode.velocity=True
        with self.assertRaises(ValueError): setpoint_message(sp,mode,now)

    def test_latest_only_and_expiration(self):
        queue=LatestQueue(); counts=Counter()
        for n in range(100): queue.put('vision',n,1)
        self.assertEqual(queue.take(counts)[1],99)
        self.assertIsNone(queue.take(counts))
        queue.put('setpoint','expired',-.1)
        self.assertIsNone(queue.take(counts))
        self.assertEqual(counts['expired_setpoint'],1)


class RouterTests(unittest.TestCase):
    def test_real_pty_partial_writes_timesync_logging_failure_and_no_replay(self):
        master,slave=os.openpty();device=os.ttyname(slave);os.close(slave)
        qgc=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);qgc.bind(('127.0.0.1',0))
        diag=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);diag.bind(('127.0.0.1',0))
        real_write=os.write
        with tempfile.TemporaryDirectory() as directory:
            router=SerialRouter(device,directory,qgc_bind=('127.0.0.1',0),
                qgc_destination=qgc.getsockname(),diag_bind=('127.0.0.1',0),
                diag_destination=diag.getsockname(),
                write_bytes=lambda fd,data:real_write(fd,data[:17]))
            router.start()
            try:
                deadline=time.monotonic()+2
                while not router.connected and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(router.connected,router.error)
                fc=mav.MAVLink(None,srcSystem=12,srcComponent=1)
                heartbeat=mav.MAVLink_heartbeat_message(2,12,0,0,3,3)
                real_write(master,heartbeat.pack(fc))
                request_stamp=123456789000
                real_write(master,mav.MAVLink_timesync_message(0,request_stamp).pack(fc))
                deadline=time.monotonic()+1
                while not router.fc_heartbeat_at and time.monotonic()<deadline:time.sleep(.01)
                now=time.time_ns()//1000
                source=vision(now)
                source.position=np.array(source.position,dtype=np.float32)
                source.q=np.array(source.q,dtype=np.float32)
                router.queue.put('vision',odometry_message(source,now),1,odometry_metadata(source))
                mode,sp=control(now)
                # A diagnostic serialization failure must not stop delivery.
                # Control is dequeued before vision, so the vision packet also
                # proves that delivery continues after this injected fault.
                router.queue.put('setpoint',setpoint_message(sp,mode,now),1,
                                 dict(injected_invalid_metadata=object()))
                router.queue.put('operator',b'EXPIRED_MUST_NOT_WRITE',-.1)
                decoder=mav.MAVLink(None);received=[];raw=bytearray()
                deadline=time.monotonic()+1.5
                while time.monotonic()<deadline:
                    ready,_,_=select.select([master],[],[],.05)
                    if ready:
                        raw.extend(os.read(master,8192))
                        packets,rest=split_packets(raw);raw=bytearray(rest)
                        received.extend(decoder.parse_char(p) for p in packets)
                types=Counter(m.get_type() for m in received if m)
                self.assertEqual(types['ODOMETRY'],1)
                self.assertEqual(types['SET_POSITION_TARGET_LOCAL_NED'],1)
                reply=next(m for m in received if m and m.get_type()=='TIMESYNC')
                self.assertEqual(reply.ts1,request_stamp)
                self.assertLess(abs(reply.tc1-time.time_ns()),2_000_000_000)
                self.assertEqual(router.counts['expired_operator'],1)
                self.assertEqual(router.counts['disconnects'],0)
                self.assertEqual(router.counts['log_errors'],1)
                self.assertTrue(router.is_alive(),router.error)
            finally:
                router.close()
            records=[json.loads(line) for line in (Path(directory)/'sent.jsonl').read_text().splitlines()]
            sent=next(record for record in records if record['kind']=='vision')
            self.assertEqual(sent['sample_us'],source.timestamp_sample)
            self.assertEqual(sent['q'],[float(v) for v in source.q])
            self.assertEqual(sent['position'],[float(v) for v in source.position])
        os.close(master);qgc.close();diag.close()


if __name__=='__main__':unittest.main()
