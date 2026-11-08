"""Operator Hold/Land after an obstacle stop; production API with inert I/O."""
import copy
import math
import time
import unittest
from unittest.mock import patch
import test_autoflight_api_offline as api


class StopRecoveryTests(unittest.TestCase):
    node = api.FlightApiTests.node

    def airborne(self, mode='failsafe'):
        n=self.node()
        n.core.mode=mode;n.core.reason='Obstacle guard: stopping path'
        n.core.was_offboard=True
        n.fc.armed=True;n.fc.landed=False;n.land=(10.,False)
        n.pos.x,n.pos.y,n.pos.z,n.pos.yaw=.2,.1,1.,.4
        n.core.cmd=[.45,.1,1.];n.core.target=[4.,0.,1.]
        n.other_setpoint_publishers=lambda:0
        return n

    def tick(self,n):
        n.core.heartbeat(10.1,'pilot');n.pos.t=n.fc.t=10.1
        with patch.object(time,'monotonic',return_value=10.1):n._tick()

    def test_hold_recovers_current_position_not_old_moving_target(self):
        for mode in ('engaged','failsafe','idle'):
            with self.subTest(mode=mode):
                n=self.airborne(mode)
                self.assertTrue(n.stop_controls(10.,'pilot')['hold'])
                ok,msg=n.do_command(10.,'pilot',{'op':'hold'})
                self.assertTrue(ok,msg)
                self.assertEqual(n.core.target,[.2,.1,1.])
                self.assertAlmostEqual(n.core.heading.yaw,.4)
                self.tick(n)
                self.assertEqual(n.core.last_out,('pose',(.2,.1,1.)))
                n.arm_pub.publish.assert_not_called()

    def test_land_after_obstacle_fault_descends_without_horizontal_shift(self):
        n=self.airborne()
        self.assertTrue(n.stop_controls(10.,'pilot')['land'])
        self.assertTrue(n.do_command(10.,'pilot',{'op':'land'})[0])
        self.assertEqual(n.core.mode,'landing')
        self.tick(n)
        self.assertEqual(n.core.last_out[0],'pose')
        self.assertEqual(n.core.last_out[1][:2],(.2,.1))
        self.assertLess(n.core.last_out[1][2],1.)
        n.arm_pub.publish.assert_not_called()

    def test_near_ground_land_uses_descent_without_climbing_to_floor(self):
        n=self.airborne();n.pos.z=.2
        self.assertFalse(n.stop_controls(10.,'pilot')['hold'])
        self.assertTrue(n.do_command(10.,'pilot',{'op':'land'})[0])
        self.assertEqual(n.core.land_phase,'touch')
        self.tick(n)
        self.assertEqual(n.core.last_out,('twist',(0.,0.,-.2)))

    def test_recovery_cancels_automatic_intent_and_never_restarts_route(self):
        for op in ('hold','land'):
            n=self.airborne()
            n.core.deferred_course={'reference':n.course.execution_reference()}
            n.core.launch_state='waiting_route'
            self.assertTrue(n.do_command(10.,'pilot',{'op':op})[0])
            self.assertIsNone(n.core.deferred_course)
            self.assertFalse(n.core.status(10.)['course_launch']['active'])
            self.tick(n)
            self.assertIsNone(n.core.course_run)

    def test_capability_and_api_agree_when_recovery_is_unavailable(self):
        changes={
            'owner':lambda n:setattr(n.core,'owner','other'),
            'stale_position':lambda n:setattr(n.pos,'t',8.),
            'invalid_position':lambda n:setattr(n.pos,'valid',False),
            'nan_position':lambda n:setattr(n.pos,'x',math.nan),
            'stale_status':lambda n:setattr(n.fc,'t',8.),
            'offboard':lambda n:setattr(n.fc,'offboard_active',False),
            'stale_landed':lambda n:setattr(n,'land',(8.,False)),
            'home':lambda n:setattr(n.core,'home',None),
            'origin':lambda n:setattr(n.fc,'origin_z',1.),
            'pilot':lambda n:setattr(n.core,'mode','pilot'),
            'fence':lambda n:setattr(n.pos,'x',3.),
            'runtime':lambda n:setattr(n.readiness,'return_value',['PX4 link lost']),
        }
        for case,change in changes.items():
            for op in ('hold','land'):
                with self.subTest(case=case,op=op):
                    n=self.airborne();change(n)
                    before=copy.deepcopy(n.core.status(10.))
                    self.assertFalse(n.stop_controls(10.,'pilot')[op])
                    self.assertFalse(n.do_command(10.,'pilot',{'op':op})[0])
                    self.assertEqual(n.core.status(10.),before)
                    n.arm_pub.publish.assert_not_called()

    def test_no_new_land_or_hold_during_landing(self):
        n=self.airborne('landing')
        for op in ('hold','land'):
            self.assertFalse(n.stop_controls(10.,'pilot')[op])
            self.assertFalse(n.do_command(10.,'pilot',{'op':op})[0])


if __name__=='__main__':unittest.main()
