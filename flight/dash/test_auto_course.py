"""Manual arm -> climb -> settled hover -> route, using synthetic telemetry."""
import math
import unittest

from dash_core import DashCore, Pos, Fc
from test_course_safety import AvoidanceTests, depth
import os as _os
from unittest.mock import patch as _patch
# These tests check the dashboard's in-flight holds, now off by default (DashParams.no_holds).
LEGACY_HOLDS = _patch.dict(_os.environ, {'DASH_LEGACY_HOLDS': '1'})


class AutoCourseTests(unittest.TestCase):
    def prepare(self, controller='smooth_position'):
        _,path=AvoidanceTests.plan(self)
        c=DashCore();c.p.require_checked_route=True;c.owner='pilot';c.owner_hb=10;c.home=(0,0,0,0)
        c.course_controller=controller
        p=Pos(10,0,0,0,0,True,0,0,0);f=Fc(10,True,False,True,True)
        self.assertTrue(c.prepare_course(10,path,6,p,f,depth())[0])
        return c,p,f,path

    def tick(self,c,p,f,now,**kwargs):
        p.t=f.t=now;c.heartbeat(now,'pilot')
        return c.step(now,p,f,kwargs.get('data',depth()))

    def test_manual_arm_starts_vertical_takeoff_then_course_once_settled(self):
        for controller in ('smooth_position','velocity_tracker'):
            c,p,f,_=self.prepare(controller)
            for i in range(1,11):
                self.tick(c,p,f,10+i*.1)
                self.assertIsNone(c.course_run);self.assertFalse(c.was_offboard)
                self.assertEqual(c.launch_state,'waiting_arm')
            f.armed=True
            for i in range(11,151):
                now=10+i*.1
                old=(p.x,p.y,p.z)
                out=self.tick(c,p,f,now)
                self.assertIsNotNone(out,c.reason)
                if c.queued_course:
                    self.assertEqual(out[0],'pose');self.assertEqual(out[1][:2],(0.,0.))
                if out[0]=='pose':p.x,p.y,p.z=out[1]
                else:p.x+=out[1][0]*.1;p.y+=out[1][1]*.1;p.z+=out[1][2]*.1
                p.vx,p.vy,p.vz=((b-a)/.1 for a,b in zip(old,(p.x,p.y,p.z)))
                p.yaw=c.heading.yaw if controller=='smooth_position' else p.yaw+c.course_yaw_rate*.1
                if p.z>.32:f.landed=False
                if c.course_run:
                    self.assertGreaterEqual(p.z,.9)
                    self.assertEqual(c.launch_state,'flying')
                    break
            self.assertIsNotNone(c.course_run)
            self.assertIsNone(c.queued_course)

    def test_hold_release_disarm_and_authority_loss_cancel_queued_start(self):
        # Operator choice: depth loss alone never cancels a course (see course_safety).
        for action in ('hold','release','land','disarm','offboard','heartbeat'):
            c,p,f,_=self.prepare();f.armed=True
            self.tick(c,p,f,10.1)
            if action=='offboard':f.offboard_active=False
            elif action=='depth':pass
            elif action=='heartbeat':pass
            else:self.assertTrue(c.command(10.1,'pilot',action,p,f)[0])
            if action=='heartbeat':c.step(11.2,p,f,depth())
            else:self.tick(c,p,f,10.2,data=depth(age=2) if action=='depth' else depth())
            self.assertIsNone(c.queued_course,action)
            self.assertIsNone(c.course_run,action)
            p.z=1;f.offboard_active=True;f.armed=True;f.landed=False
            for i in range(30):self.tick(c,p,f,12+i*.1)
            self.assertIsNone(c.course_run,action)

    def test_cannot_prepare_with_bad_plan_velocity_health_or_launch_position(self):
        for change in ('height','xy','radius','depth','unknown_velocity','armed','stale'):
            c,p,f,path=self.prepare();c.stop_course(10);c.mode='idle'
            radius=6;data=depth()
            if change=='height':path['points'][0][2]=3
            if change=='xy':p.x=.2
            if change=='radius':radius=1
            if change=='depth':data=depth(age=2)
            if change=='unknown_velocity':p.vx=None
            if change=='armed':f.armed=True
            if change=='stale':p.t=8
            self.assertFalse(c.prepare_course(10,path,radius,p,f,data)[0],change)
            self.assertIsNone(c.queued_course)

    @LEGACY_HOLDS
    def test_no_horizontal_start_with_unsettled_vertical_velocity_or_takeoff_timeout(self):
        c,p,f,_=self.prepare();f.armed=True;f.landed=False;p.z=1;p.vz=.2
        for i in range(1,320):
            self.tick(c,p,f,10+i*.1)
            self.assertIsNone(c.course_run)
        self.assertEqual(c.mode,'failsafe')
        self.assertIn('30 seconds',c.reason)


if __name__=='__main__':unittest.main()
