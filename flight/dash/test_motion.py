"""Synthetic command tests. No HTTP request or command goes to the live drone."""
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from dash_core import DashCore, DashParams, Pos, Fc
import os as _os
from unittest.mock import patch as _patch
# These tests check the dashboard's in-flight holds, now off by default (DashParams.no_holds).
LEGACY_HOLDS = _patch.dict(_os.environ, {'DASH_LEGACY_HOLDS': '1'})


def prepared():
    limits=json.loads((Path(__file__).parent/'flight_limits.json').read_text())
    core=DashCore(DashParams(min_alt_m=limits['min_altitude_m'],max_alt_m=limits['max_altitude_m']))
    pos=Pos(10,4,-3,-.8,math.pi/2,vx=0.,vy=0.)
    fc=Fc(10,True,False,True,True,origin_z=-.8)
    assert core.command(10,'pilot','claim')[0]
    assert core.command(10,'pilot','engage',pos=pos,fc=fc,alt=2)[0]
    return core,pos,fc


class MotionTests(unittest.TestCase):
    def test_two_metres_accepted_and_outside_limit_rejected_without_changing_target(self):
        c,p,f=prepared()
        self.assertAlmostEqual(c.target[2]-c.ground_z,2)
        before=list(c.target)
        for alt in (2.01,.29,math.nan,math.inf,True,None,'2'):
            self.assertFalse(c.command(10,'pilot','alt',pos=p,fc=f,alt=alt)[0])
            self.assertEqual(c.target,before)
        self.assertTrue(c.command(10,'pilot','alt',pos=p,fc=f,alt=1.7)[0])
        self.assertAlmostEqual(c.target[2]-c.ground_z,1.7)

    def test_out_of_range_prepare_does_not_start_a_clamped_hover(self):
        c=DashCore();p=Pos(10,0,0,0,0);f=Fc(10,True,False,True,True)
        c.command(10,'pilot','claim')
        ok,msg=c.command(10,'pilot','engage',pos=p,fc=f,alt=3)
        self.assertFalse(ok);self.assertIn('2 m',msg)
        self.assertEqual(c.mode,'idle');self.assertIsNone(c.step(10,p,f))

    def test_absolute_home_point_rotates_once_and_keeps_height_and_ramp(self):
        c,p,f=prepared();f.armed=True;f.landed=False
        c.step(10,p,f)
        before=list(c.cmd)
        for _ in range(2):
            self.assertTrue(c.command(10,'pilot','goto',pos=p,fc=f,forward=1,left=.5)[0])
            self.assertEqual(c.cmd,before)
            self.assertAlmostEqual(c.target[0],3.5)
            self.assertAlmostEqual(c.target[1],-2)
            self.assertAlmostEqual(c.target[2],1.2)
        p.t=f.t=10.1;c.heartbeat(10.1,'pilot')
        out=c.step(10.1,p,f)
        self.assertLessEqual(math.hypot(out[1][0]-before[0],out[1][1]-before[1]),.1000001)
        self.assertLessEqual(abs(out[1][2]-before[2]),.0300001)

    def test_one_metre_horizontal_ramp_is_vector_limited_with_unchanged_climb(self):
        c,p,f=prepared();c.p=DashParams(v_xy=1.0)
        f.armed=True;f.landed=False
        c.step(10,p,f)
        self.assertTrue(c.command(10,'pilot','goto',pos=p,fc=f,forward=1,left=1)[0])
        # Establish measured alignment before checking the translation ramp.
        p.yaw=math.atan2(c.target[1]-c.cmd[1],c.target[0]-c.cmd[0])
        c.heading.reset(p.yaw)
        start=10.
        for tick in range(1,11):
            start=10+tick*.1;p.t=f.t=start;c.heartbeat(start,'pilot')
            c.step(start,p,f);p.x,p.y=c.cmd[:2]
            if c.heading.phase=='aligned':break
        self.assertEqual(c.heading.phase,'aligned')
        for tick in range(1,11):
            before=list(c.cmd)
            now=start+tick*.1;p.t=f.t=now;c.heartbeat(now,'pilot')
            out=c.step(now,p,f)
            self.assertAlmostEqual(math.hypot(out[1][0]-before[0],out[1][1]-before[1]),.1)
            self.assertAlmostEqual(out[1][2]-before[2],.03)
            p.x,p.y=c.cmd[:2]
        self.assertEqual(c.p.land_v,.2)
        self.assertEqual(c.status(now)['params']['v_xy'],1.0)

    def test_horizontal_speed_rejects_nonfinite_and_nonpositive_configuration(self):
        for value in (math.nan,math.inf,-math.inf,0,-.1,True):
            with self.assertRaises(ValueError):DashParams(v_xy=value)

    def test_goto_rejects_bad_point_bounds_state_freshness_and_owner(self):
        cases=[dict(forward=2,left=0),dict(forward=1.1,left=1.1),dict(forward=math.nan,left=0),
               dict(forward=0,left=math.inf),dict(forward=True,left=0),dict(forward=0),dict(left=0)]
        for values in cases:
            c,p,f=prepared();f.armed=True;f.landed=False;before=list(c.target)
            self.assertFalse(c.command(10,'pilot','goto',pos=p,fc=f,**values)[0])
            self.assertEqual(c.target,before)
        for change in (lambda c,p,f:setattr(f,'armed',False),lambda c,p,f:setattr(f,'landed',True),
                       lambda c,p,f:setattr(f,'offboard_active',False),lambda c,p,f:setattr(f,'t',8),
                       lambda c,p,f:setattr(p,'t',9),lambda c,p,f:setattr(p,'valid',False),
                       lambda c,p,f:setattr(c,'mode','landing'),lambda c,p,f:setattr(c,'owner','viewer')):
            c,p,f=prepared();f.armed=True;f.landed=False;before=list(c.target);change(c,p,f)
            self.assertFalse(c.command(10,'pilot','goto',pos=p,fc=f,forward=.5,left=0)[0])
            self.assertEqual(c.target,before)

    def test_goto_checks_fence_around_engage_point_even_when_home_differs(self):
        c,p,f=prepared();f.armed=True;f.landed=False
        c.home=(4,-5,-.8,math.pi/2)
        self.assertFalse(c.command(10,'pilot','goto',pos=p,fc=f,forward=0,left=0)[0])
        self.assertTrue(c.command(10,'pilot','goto',pos=p,fc=f,forward=2,left=0)[0])
        self.assertAlmostEqual(c.target[1],-3)

    @LEGACY_HOLDS
    def test_goto_stops_on_lost_heartbeat_and_does_not_resume_itself(self):
        c,p,f=prepared();f.armed=True;f.landed=False
        c.command(10,'pilot','goto',pos=p,fc=f,forward=1,left=0)
        c.step(10,p,f);p.t=f.t=11.1
        self.assertIsNone(c.step(11.1,p,f));self.assertEqual(c.mode,'failsafe')
        c.heartbeat(11.2,'pilot')
        self.assertIsNone(c.step(11.2,p,f))

    def test_api_forwards_point_and_checks_current_flight_health(self):
        from dash_node import DashNode
        c,p,f=prepared();f.armed=True;f.landed=False
        n=NS(core=c,pos=p,fc=f,a=NS(live=True),lock=threading.Lock(),
             readiness=Mock(return_value=[]),get_logger=lambda:Mock(),arm_pub=Mock())
        self.assertTrue(DashNode.do_command(n,10,'pilot',dict(op='goto',forward=1,left=.5))[0])
        n.readiness.assert_called_with(10,preflight=False)
        self.assertAlmostEqual(c.target[0],3.5)
        before=list(c.target);n.readiness.return_value=['PX4 local position missing or stale']
        self.assertFalse(DashNode.do_command(n,10,'pilot',dict(op='goto',forward=0,left=0))[0])
        self.assertEqual(c.target,before);n.arm_pub.publish.assert_not_called()

    def test_http_command_reaches_point_handler_without_any_real_publishers(self):
        from dash_node import DashNode,make_handler
        from http.server import ThreadingHTTPServer
        import time
        import urllib.request
        c,p,f=prepared();f.armed=True;f.landed=False
        p.t=f.t=time.monotonic()
        n=NS(core=c,pos=p,fc=f,a=NS(live=True),lock=threading.Lock(),
             readiness=Mock(return_value=[]),get_logger=lambda:Mock(),arm_pub=Mock())
        n.do_command=lambda now,client,body:DashNode.do_command(n,now,client,body)
        n.snapshot=lambda now,client:c.status(now)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(n,'test-only-token','test page'))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            request=urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/cmd?k=test-only-token',
                data=json.dumps(dict(c='pilot',op='goto',forward=1,left=.5)).encode(),
                headers={'Content-Type':'application/json'},method='POST')
            with urllib.request.urlopen(request,timeout=2) as response:result=json.load(response)
            self.assertTrue(result['ok'],result)
            self.assertEqual(result['state']['target'],[3.5,-2,1.2])
            n.arm_pub.publish.assert_not_called()
        finally:
            server.shutdown();thread.join(2);server.server_close()


class WireTests(unittest.TestCase):
    def test_two_metre_hover_and_point_survive_interface_and_mavlink_conversion(self):
        import os
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID'),'12','Run isolated from live DDS')
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from px4_msgs.msg import VehicleStatus,VehicleLocalPosition,VehicleLandDetected
        from skybeta_flight_interface.flight_interface import FlightInterface
        import sys
        sys.path.insert(0,'/root/team_ws/claude/mavlink_usb')
        from conversion import setpoint_message
        from pymavlink.dialects.v20 import common as mav
        rclpy.init(args=['--ros-args','-p','max_altitude_m:=2.0','-p','max_speed_mps:=1.0',
                        '-p','use_px4_height:=true','-p','target_system:=12'])
        n=FlightInterface()
        try:
            records={'control':[],'setpoint':[],'command':[]}
            n.offboard_pub.publish=records['control'].append
            n.setpoint_pub.publish=records['setpoint'].append
            n.command_pub.publish=records['command'].append
            status=VehicleStatus();status.arming_state=1;status.nav_state=14;n.status_cb(status)
            land=VehicleLandDetected();land.landed=True;n.land_detected_cb(land)
            local=VehicleLocalPosition();local.x=-3.;local.y=4.;local.z=.8
            local.xy_valid=True;local.z_valid=True;n.local_position_cb(local)
            c,p,f=prepared();f.armed=True;f.landed=False
            for index in range(80):
                t=10+index*.1;p.t=f.t=t;c.heartbeat(t,'pilot')
                out=c.step(t,p,f)
            for target,expected in ((out[1],(-3,4,-1.2)),):
                msg=PoseStamped();msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=target
                n.pose_setpoint_cb(msg);n.tick()
                self.assertFalse(n.last_clamp_reasons)
                wire=setpoint_message(records['setpoint'][-1],records['control'][-1],records['setpoint'][-1].timestamp)
                decoded=mav.MAVLink(None).parse_char(wire.pack(mav.MAVLink(None)))
                for actual,want in zip((decoded.x,decoded.y,decoded.z),expected):self.assertAlmostEqual(actual,want,places=5)
            self.assertTrue(c.command(t,'pilot','goto',pos=p,fc=f,forward=1,left=.5)[0])
            for index in range(1,61):
                now=t+index*.1;p.t=f.t=now;c.heartbeat(now,'pilot');out=c.step(now,p,f)
                p.x,p.y=c.cmd[:2];p.yaw=c.heading.yaw
            msg=PoseStamped();msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=out[1]
            n.pose_setpoint_cb(msg);n.tick()
            wire=setpoint_message(records['setpoint'][-1],records['control'][-1],records['setpoint'][-1].timestamp)
            decoded=mav.MAVLink(None).parse_char(wire.pack(mav.MAVLink(None)))
            for actual,want in zip((decoded.x,decoded.y,decoded.z),(-2,3.5,-1.2)):self.assertAlmostEqual(actual,want,places=5)
            self.assertEqual(records['command'],[])
        finally:
            n.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
