"""Vertical takeoff with deferred guarded course; inert production API/timer."""
import copy
import math
import time
import unittest
from unittest.mock import patch

import test_autoflight_api_offline as api
import os as _os
from unittest.mock import patch as _patch
# These tests check the dashboard's in-flight holds, now off by default (DashParams.no_holds).
LEGACY_HOLDS = _patch.dict(_os.environ, {'DASH_LEGACY_HOLDS': '1'})


class TakeoffThenCourseTests(unittest.TestCase):
    node = api.FlightApiTests.node

    def launch(self, depth=None, controller='smooth_position'):
        n = self.node()
        n.data = depth or dict(status='live', age_s=.1, cell_m=.08, points=[])
        n.course.update_preview(n.data, now=10.)
        n.core.course_controller = controller
        n.fc_raw = dict(velocity_timeout_hold_current=True, envelope={'max_speed_mps': 1.})
        ok, message = n.do_command(10., 'pilot', {'op': 'launch', 'flight': 'course'})
        self.assertTrue(ok, message)
        self.assertIsNotNone(n.core.deferred_course)
        self.assertIsNone(n.core.queued_course)
        self.assertIsNone(n.core.course_run)
        n.arm_pub.publish.assert_called_once()
        self.assertEqual(n.arm_pub.publish.call_args.args[0].param2, 0.)
        n.fc.armed = True
        return n

    def tick(self, n, now, integrate=True, heartbeat=True):
        n.pos.t = n.fc.t = now
        n.land = (now, n.fc.landed)
        if heartbeat: n.core.heartbeat(now, 'pilot')
        with patch.object(time, 'monotonic', return_value=now): n._tick()
        out = n.core.last_out
        if integrate and out is not None:
            old = (n.pos.x,n.pos.y,n.pos.z)
            if out[0] == 'pose': n.pos.x,n.pos.y,n.pos.z = out[1]
            else:
                n.pos.x += out[1][0]*.1
                n.pos.y += out[1][1]*.1
                n.pos.z += out[1][2]*.1
            n.pos.vx,n.pos.vy,n.pos.vz = ((b-a)/.1 for a,b in zip(old,(n.pos.x,n.pos.y,n.pos.z)))
            n.fc.landed = n.pos.z <= .32
        return out

    def settle(self, n):
        for i in range(1,71):
            now = 10.+i*.1
            out = self.tick(n,now)
            self.assertIsNone(n.core.course_run)
            self.assertEqual(out[0], 'pose')
            self.assertEqual(out[1][:2], (0.,0.))
            if n.core.launch_state == 'waiting_route':
                return now
        self.fail(n.core.reason or n.core.launch_reason)

    def clear_route(self, n, now):
        n.data = dict(status='live', age_s=.1, cell_m=.08, points=[[2.,0.]])
        n.course.update_preview(n.data, now=now)

    def test_empty_stale_unscaled_and_far_blocked_routes_allow_only_vertical_takeoff(self):
        conditions = [
            dict(status='live',age_s=.1,cell_m=.08,points=[]),
            dict(status='live',age_s=2.,cell_m=.08,points=[[2.,0.]]),
            dict(status='unscaled',points=[]),
            dict(status='live',age_s=.1,cell_m=.08,points=[[2.,y/10] for y in range(-12,13)]),
        ]
        for data in conditions:
            with self.subTest(depth=data):
                n = self.launch(data)
                now = self.settle(n)
                self.assertEqual(n.core.launch_state,'waiting_route')
                self.assertTrue(n.core.status(now)['course_launch']['active'])
                self.assertAlmostEqual(n.pos.z,1.)
                for i in range(1,4):
                    out = self.tick(n,now+i*.1)
                    self.assertEqual(out,('pose',(0.,0.,1.)))
                    self.assertIsNone(n.core.course_run)
                n.arm_pub.publish.assert_called_once()

    def test_fresh_route_starts_once_after_settled_hover_for_both_followers(self):
        for controller in ('smooth_position','velocity_tracker'):
            with self.subTest(controller=controller):
                n = self.launch(controller=controller)
                now = self.settle(n)
                self.clear_route(n,now+.1)
                self.tick(n,now+.1)
                run = n.core.course_run
                self.assertIsNotNone(run,n.core.launch_reason)
                self.assertIsNotNone(run.avoidance)
                self.assertIsNone(n.core.deferred_course)
                self.assertEqual(n.core.launch_state,'flying')
                self.assertEqual(run.reference,n.course.execution_reference())
                for i in range(2,5):
                    self.tick(n,now+i*.1)
                    self.assertIs(n.core.course_run,run)
                n.arm_pub.publish.assert_called_once()

    def test_valid_route_before_takeoff_does_not_skip_vertical_settling(self):
        for controller in ('smooth_position','velocity_tracker'):
            n = self.launch(controller=controller)
            for i in range(1,16):
                now = 10.+i*.1
                self.clear_route(n,now)
                out = self.tick(n,now)
                self.assertIsNone(n.core.course_run)
                self.assertEqual(out[0],'pose')
                self.assertEqual(out[1][:2],(0.,0.))
                self.assertLess(n.pos.z,.9)

    def test_takeoff_seen_obstacles_survive_course_promotion(self):
        n = self.launch(dict(status='live',age_s=.1,cell_m=.08,points=[[1.,.5]]))
        # Keep the route unavailable without discarding the valid column observations.
        n.course._adjustment_status = 'blocked'
        now = self.settle(n)
        self.clear_route(n,now+.1)
        self.tick(n,now+.1)
        self.assertIsNotNone(n.core.course_run,n.core.launch_reason)
        self.assertIn((1.,.5),n.core.course_run.obstacle_guard.cells)

    def test_observed_occupied_column_brakes_and_cancels_without_autoresume(self):
        n = self.launch()
        self.tick(n,10.1)
        self.tick(n,10.2)
        before = (n.pos.x,n.pos.y,n.pos.z)
        n.data = dict(status='live',age_s=.1,cell_m=.08,points=[[0.,0.]])
        out = self.tick(n,10.3,integrate=False)
        self.assertEqual(out,('pose',before))
        self.assertEqual(n.core.mode,'failsafe')
        self.assertIsNone(n.core.deferred_course)
        n.freeze_interface.assert_called_once()
        self.clear_route(n,10.4)
        self.assertIsNone(self.tick(n,10.4))
        self.assertIsNone(n.core.course_run)

    @LEGACY_HOLDS
    def test_manual_actions_and_faults_cancel_deferred_course_permanently(self):
        actions = ('hold','land','release','disarm','arm','nudge','goto','alt','owner',
                   'course_stop','heartbeat','position','health','authority','disarmed',
                   'reference','landed')
        for action in actions:
            with self.subTest(action=action):
                n = self.launch()
                now = self.settle(n)
                if action in ('hold','land','release','disarm','arm','nudge','goto','alt'):
                    kw = {'nudge':{'fwd':.1},'goto':{'forward':0.,'left':0.},'alt':{'alt':1.}}.get(action,{})
                    self.assertTrue(n.core.command(now,'pilot',action,n.pos,n.fc,**kw)[0])
                elif action == 'owner':
                    n.core.owner_hb = now-4.
                    self.assertTrue(n.core.command(now,'new_owner','claim',n.pos,n.fc)[0])
                elif action == 'course_stop': self.assertTrue(n.do_course(now,'pilot',{'op':'stop'})[0])
                elif action == 'heartbeat': n.core.owner_hb = now-2.
                elif action == 'position': n.pos.valid = False
                elif action == 'health': n.readiness.return_value = ['VIO lost']
                elif action == 'authority': n.fc.offboard_active = False
                elif action == 'disarmed': n.fc.armed = False
                elif action == 'reference': n.course.invalidate_anchor('test reset')
                elif action == 'landed': n.fc.landed = True
                self.tick(n,now+.1,integrate=False,heartbeat=action!='heartbeat')
                self.assertIsNone(n.core.deferred_course)
                n.pos.valid = True
                n.readiness.return_value = []
                n.fc.armed,n.fc.landed,n.fc.offboard_active = True,False,True
                self.clear_route(n,now+.2)
                for i in range(2,5): self.tick(n,now+i*.1,integrate=False)
                self.assertIsNone(n.core.course_run)

    @LEGACY_HOLDS
    def test_waiting_route_has_no_extra_timeout_but_unsettled_takeoff_has_one(self):
        n = self.launch()
        self.settle(n)
        self.tick(n,45.)
        self.assertEqual(n.core.mode,'engaged')
        self.assertEqual(n.core.launch_state,'waiting_route')
        n = self.launch()
        for i in range(1,315): self.tick(n,10.+i*.1,integrate=False)
        self.assertEqual(n.core.mode,'failsafe')
        self.assertIn('30 seconds',n.core.reason)
        self.assertIsNone(n.core.deferred_course)

    def test_unchecked_or_different_reference_cannot_be_promoted(self):
        n = self.launch()
        now = self.settle(n)
        raw = n.course.original_path()
        self.assertFalse(n.core.start_deferred_course(now,raw,n.pos,n.fc,n.data)[0])
        self.clear_route(n,now)
        path = n.course.execution_path(now)
        path['reference'] = ('wrong',)
        self.assertFalse(n.core.start_deferred_course(now,path,n.pos,n.fc,n.data)[0])
        self.assertIsNone(n.core.course_run)
        self.assertIsNotNone(n.core.deferred_course)

    def test_manual_arm_opt_out_cancels_deferred_intent(self):
        n = self.launch()
        now = self.settle(n)
        self.assertTrue(n.do_command(now,'pilot',{'op':'arm','auto_course':False})[0])
        self.assertIsNone(n.core.deferred_course)
        self.clear_route(n,now+.1)
        self.tick(n,now+.1)
        self.assertIsNone(n.core.course_run)


if __name__ == '__main__': unittest.main()
