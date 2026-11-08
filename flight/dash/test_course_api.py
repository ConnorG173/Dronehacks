"""Synthetic course API tests; no ROS node, real endpoint or flight commands."""
import copy
import math
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from course_state import CourseState
from dash_core import DashCore,Pos,Fc
from dash_node import DashNode


class CourseApiTests(unittest.TestCase):
    def node(self,path):
        core=DashCore();core.owner='pilot'
        return SimpleNamespace(core=core,lock=threading.Lock(),course=CourseState(path),
            pos=Pos(10,4,-3,-.8,math.pi/2),fc=Fc(10,True,False,True,True),
            depth_overlay=SimpleNamespace(snapshot=lambda now=None: dict(
                status='live',age_s=.1,cell_m=.08,points=[[20.,20.]])),
            land=(10,True),arm_pub=Mock(),pub_pose=Mock(),pub_twist=Mock(),freeze_interface=Mock())

    def test_start_stop_use_core_without_arming_or_direct_publish(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp)/'course.json')
            n.a=SimpleNamespace(live=True)
            n.readiness=Mock(return_value=[])
            n.core.home=(4,-3,-.8,math.pi/2)
            n.course.import_route([[0,0,1],[2,0,1]])
            n.course.capture_anchor([4,-3,-.8],math.pi/2)
            n.course.update_preview(n.depth_overlay.snapshot(),now=10)
            n.core.mode='engaged';n.core.was_offboard=True
            n.core.ground_z=n.core.origin_z=-.8
            n.core.cmd=[4,-3,.2];n.core.target=list(n.core.cmd)
            n.core.heading.reset(math.pi/2)
            n.pos.z=.2;n.pos.vx=n.pos.vy=0.
            n.fc.armed=True;n.fc.landed=False
            self.assertFalse(DashNode.do_course(n,10,'viewer',{'op':'start','radius_m':6})[0])
            self.assertFalse(DashNode.do_course(n,10,'pilot',{'op':'start'})[0])
            self.assertTrue(DashNode.do_course(n,10,'pilot',{'op':'start','radius_m':6})[0])
            self.assertTrue(n.core.course_run.active)
            self.assertFalse(DashNode.do_course(n,10,'pilot',{'op':'import','trajectory':[[0,0,1],[1,0,1]]})[0])
            self.assertFalse(DashNode.do_course(n,10,'pilot',{'op':'start','radius_m':6})[0])
            self.assertTrue(DashNode.do_course(n,10,'pilot',{'op':'stop'})[0])
            self.assertFalse(n.core.course_run.active)
            self.assertEqual(n.core.target,n.core.cmd)
            for pub in (n.arm_pub,n.pub_pose,n.pub_twist):pub.publish.assert_not_called()

    def test_px4_frame_reset_invalidates_course_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp)/'course.json');n.last_course_sample_us=None
            n.course.import_route([[0,0,1],[1,0,1]])
            n.course.capture_anchor([4,-3,-.8],math.pi/2)
            n.pos=None
            m=SimpleNamespace(x=-3.,y=4.,z=.8,heading=0.,xy_valid=True,z_valid=True,
                dist_bottom=0.,dist_bottom_valid=False,dist_bottom_sensor_bitfield=1,
                timestamp_sample=100,xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0)
            DashNode._lpos(n,m)
            self.assertIsNotNone(n.course.execution_reference())
            m.xy_reset_counter=1;m.timestamp_sample=200
            DashNode._lpos(n,m)
            self.assertIsNone(n.course.execution_reference())
            self.assertIsNone(n.core.home)

    def test_import_anchor_preview_never_change_flight_state_or_publish(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp)/'course.json')
            before=(copy.deepcopy(n.core.target),copy.deepcopy(n.core.cmd),n.core.home,n.core.mode)
            for body in ({'op':'import','trajectory':[[0,0,1],[1,0,1]]},
                         {'op':'capture_anchor'},{'op':'preview'}):
                self.assertTrue(DashNode.do_course(n,10,'pilot',body)[0])
            self.assertEqual((n.core.target,n.core.cmd,n.core.mode),(before[0],before[1],before[3]))
            self.assertEqual(n.core.home,(4,-3,-.8,math.pi/2))
            for pub in (n.arm_pub,n.pub_pose,n.pub_twist):pub.publish.assert_not_called()
            n.freeze_interface.assert_not_called()
            anchor=n.course.snapshot()['anchor']
            self.assertEqual(anchor['origin_enu'],[4,-3,-.8])
            self.assertAlmostEqual(anchor['yaw_rad'],math.pi/2)

    def test_reset_home_and_route_import_use_the_same_course_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp)/'course.json');n.get_logger=lambda:Mock()
            n.a=SimpleNamespace(live=False)
            n.course.import_route([[0,0,1],[1,0,1]])
            self.assertTrue(DashNode.do_command(n,10,'pilot',{'op':'reset'})[0])
            anchor=n.course.snapshot()['anchor']
            self.assertEqual(anchor['origin_enu'],list(n.core.home[:3]))
            self.assertEqual(anchor['yaw_rad'],n.core.home[3])
            n.pos.x=5.;n.pos.yaw=-.4
            self.assertTrue(DashNode.do_command(n,10,'pilot',{'op':'reset'})[0])
            anchor=n.course.snapshot()['anchor']
            self.assertEqual(anchor['origin_enu'],[5.,-3,-.8])
            self.assertEqual(anchor['yaw_rad'],-.4)
            self.assertTrue(DashNode.do_course(n,10,'pilot',{'op':'import','trajectory':[[0,0,1],[0,1,1]]})[0])
            self.assertEqual(n.course.snapshot()['anchor']['origin_enu'],list(n.core.home[:3]))
            for pub in (n.arm_pub,n.pub_pose,n.pub_twist):pub.publish.assert_not_called()

    def test_capture_requires_owner_and_fresh_disarmed_landed_position(self):
        changes=[lambda n:setattr(n.fc,'armed',True),lambda n:setattr(n.fc,'landed',False),
                 lambda n:setattr(n.fc,'t',8),lambda n:setattr(n.fc,'px4_link',False),
                 lambda n:setattr(n.pos,'t',9),lambda n:setattr(n.pos,'valid',False),
                 lambda n:setattr(n.pos,'yaw',math.nan),lambda n:setattr(n,'land',(8,True)),
                 lambda n:setattr(n,'land',(10,False))]
        with tempfile.TemporaryDirectory() as tmp:
            for change in changes:
                n=self.node(Path(tmp)/'course.json');change(n)
                self.assertFalse(DashNode.do_course(n,10,'pilot',{'op':'capture_anchor'})[0])
                self.assertIsNone(n.course.snapshot()['anchor'])
            n=self.node(Path(tmp)/'course.json')
            for client in ('viewer','','bad id'):
                self.assertFalse(DashNode.do_course(n,10,client,{'op':'import','trajectory':[[0,0,1],[1,0,1]]})[0])
            self.assertFalse((Path(tmp)/'course.json').exists())

    def test_http_course_body_limit_is_separate_and_auth_required(self):
        from http.server import ThreadingHTTPServer
        from urllib.request import Request,urlopen
        from urllib.error import HTTPError
        import json
        from dash_node import make_handler
        node=SimpleNamespace(do_course=Mock(return_value=(True,'saved')),
            do_command=Mock(),snapshot=lambda now,client:{})
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(node,'test-token','page'))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        payload=json.dumps({'c':'pilot','op':'import','trajectory':{'waypoints':[[0,0,1]]*1000}}).encode()
        base='http://127.0.0.1:'+str(server.server_address[1])
        try:
            request=Request(base+'/api/course?k=test-token',data=payload,headers={'Content-Type':'application/json'})
            with urlopen(request,timeout=2) as response:
                self.assertTrue(json.load(response)['ok'])
            node.do_course.assert_called_once()
            for endpoint,code in (('/api/course?k=bad',403),('/api/cmd?k=test-token',400)):
                with self.assertRaises(HTTPError) as error:urlopen(Request(base+endpoint,data=payload),timeout=2)
                self.assertEqual(error.exception.code,code)
                error.exception.close()
            node.do_command.assert_not_called()
        finally:
            server.shutdown();server.server_close();thread.join(2)


if __name__=='__main__':unittest.main()
