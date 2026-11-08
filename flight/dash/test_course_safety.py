"""Synthetic obstacles and command sweeps; no hardware or vehicle publishers."""
import math
from pathlib import Path
import tempfile
import unittest

from course_safety import check_command, observations, segment_clearance, ObstacleGuard
from course_state import CourseState
from dash_core import DashCore, Pos, Fc


def depth(points=None, age=.1):
    return dict(status='live', age_s=age, cell_m=.08,
                points=[[2.,0.]] if points is None else points)


class AvoidanceTests(unittest.TestCase):
    def plan(self, now=10):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        state=CourseState(Path(temp.name)/'course.json')
        self.assertTrue(state.import_route([[0,0,1],[4,0,1]])[0])
        self.assertTrue(state.capture_anchor([0,0,0],0)[0])
        state.update_preview(depth(),now=now)
        return state,state.execution_path(now)

    def test_execution_uses_full_detour_and_rejects_unvalidated_or_stale_plan(self):
        state,path=self.plan()
        self.assertEqual(path['source'],'adjusted')
        self.assertGreater(max(abs(p[1]) for p in path['points']),.4)
        self.assertEqual(path['points'][0],[0,0,1])
        self.assertEqual(path['points'][-1],[4,0,1])
        walls,cell,_=observations(depth())
        self.assertGreaterEqual(min(segment_clearance(a,b,walls,cell)
            for a,b in zip(path['points'],path['points'][1:])),.4-1e-9)
        path['points'][0][0]=99
        self.assertEqual(state.execution_path(10)['points'][0][0],0)
        with self.assertRaises(ValueError):state.execution_path(11.401)
        # A fresh scaled frame with no wall cells plans the route unchanged.
        state.update_preview(depth(points=[]),now=12)
        self.assertEqual(state.execution_path(12)['source'],'unchanged')
        state.update_preview(depth(points=[[0,0]]),now=13)
        with self.assertRaises(ValueError):state.execution_path(13)
        self.assertEqual(state.document['waypoints'],[[0,0,1],[4,0,1]])

    def test_full_segment_and_measured_stopping_path_are_checked(self):
        cases=[((0,0,1),(0,0),('pose',(4,0,1)),(0,0,1)),
               ((1.3,0,1),(.8,0),('twist',(0,0,0)),(1.3,0,1)),
               ((1.3,0,1),(0,0),('twist',(.8,0,0)),(1.3,0,1))]
        for pos,vel,out,old in cases:
            with self.assertRaises(ValueError):check_command(depth(),pos,vel,out,old,.1,.3)
        check_command(depth(),(1,1,1),(.1,0),('pose',(1.01,1,1)),(1,1,1),.1,.3)

    def test_invalid_or_unknown_depth_fails_closed(self):
        self.assertEqual(observations(depth([]))[0].shape,(0,2))
        for data in (None,{},depth(age=1.501),depth(age=-.01),depth(age=math.nan),
                     dict(depth(),cell_m=True),depth([[math.nan,0]]),depth([[True,0]]),
                     dict(depth(),status='unscaled')):
            with self.assertRaises(ValueError):observations(data)

    def test_turning_away_never_erases_previously_observed_obstacles(self):
        guard=ObstacleGuard(dict(clearance_m=.3,walls=[[2,0]],cell_m=.08))
        # Only a different wall is visible now, but the original wall remains.
        with self.assertRaises(ValueError):
            guard.check(depth(points=[[20,20]]),(0,0,1),(0,0),('pose',(4,0,1)),(0,0,1),.1)
        guard.check(depth(points=[[20,20]]),(0,1,1),(.1,0),('pose',(.01,1,1)),(0,1,1),.1)

    def test_both_followers_fly_detour_without_intersecting_supplied_wall(self):
        for controller in ('smooth_position','velocity_tracker'):
            with self.subTest(controller=controller):
                _,path=self.plan()
                c=DashCore();c.owner='pilot';c.owner_hb=10;c.home=(0,0,0,0)
                c.mode='engaged';c.was_offboard=True;c.cmd=[0,0,1];c.target=list(c.cmd)
                c.heading.reset(0);c.last_step=10;c.course_controller=controller
                p=Pos(10,0,0,1,0,True,0,0,0);f=Fc(10,True,True,True,False)
                self.assertTrue(c.start_course(10,path,6,p,f)[0])
                peak=0;walls,cell,_=observations(depth())
                for tick in range(1,2501):
                    now=10+.1*tick;p.t=f.t=now;c.heartbeat(now,'pilot')
                    old=(p.x,p.y,p.z)
                    out=c.step(now,p,f,depth())
                    self.assertIsNotNone(out,c.reason)
                    if out[0]=='pose':
                        p.x,p.y,p.z=out[1];p.yaw=c.heading.yaw
                    else:
                        p.x+=out[1][0]*.1;p.y+=out[1][1]*.1;p.z+=out[1][2]*.1
                        p.yaw+=c.course_yaw_rate*.1
                    p.vx,p.vy,p.vz=((v-a)/.1 for a,v in zip(old,(p.x,p.y,p.z)))
                    peak=max(peak,abs(p.y))
                    self.assertGreaterEqual(segment_clearance(old,(p.x,p.y),walls,cell),.3-1e-8)
                    self.assertLessEqual(math.hypot(p.vx,p.vy),.35+1e-6)
                    if not c.course_run.active:break
                self.assertEqual(c.course_run.state,'complete',c.course_run.status())
                self.assertGreater(peak,.4)

    def test_depth_loss_keeps_flying_the_course(self):
        for controller in ('smooth_position','velocity_tracker'):
            _,path=self.plan()
            c=DashCore();c.owner='pilot';c.owner_hb=10;c.home=(0,0,0,0)
            c.mode='engaged';c.was_offboard=True;c.cmd=[0,0,1];c.target=list(c.cmd)
            c.heading.reset(0);c.last_step=10;c.course_controller=controller
            p=Pos(10,0,0,1,0,True,0,0,0);f=Fc(10,True,True,True,False)
            self.assertTrue(c.start_course(10,path,6,p,f)[0])
            for tick in range(1,40):  # 4 s of stale depth
                now=10+.1*tick;p.t=f.t=now;c.heartbeat(now,'pilot')
                self.assertIsNotNone(c.step(now,p,f,depth(age=2)),c.reason)
            self.assertEqual(c.mode,'engaged',c.reason)
            self.assertIsNone(c.paused_course)
            self.assertTrue(c.course_run.active)

    def test_contact_with_an_obstacle_cell_still_cancels_without_resume(self):
        for controller in ('smooth_position','velocity_tracker'):
            for bad in (depth(points=[[0,0]]),):
                _,path=self.plan()
                c=DashCore();c.owner='pilot';c.owner_hb=10;c.home=(0,0,0,0)
                c.mode='engaged';c.was_offboard=True;c.cmd=[0,0,1];c.target=list(c.cmd)
                c.heading.reset(0);c.last_step=10;c.course_controller=controller
                p=Pos(10,0,0,1,0,True,0,0,0);f=Fc(10,True,True,True,False)
                self.assertTrue(c.start_course(10,path,6,p,f)[0])
                self.assertEqual(c.step(10.1,p,f,bad),('pose',(p.x,p.y,p.z)))
                self.assertEqual(c.mode,'failsafe');self.assertFalse(c.course_run.active)
                p.t=f.t=10.2;c.heartbeat(10.2,'pilot')
                self.assertIsNone(c.step(10.2,p,f,depth()))


class GuardFixTests(unittest.TestCase):
    def test_depth_gap_uses_memory_and_never_raises_in_flight(self):
        g=ObstacleGuard(dict(clearance_m=.3,walls=[[1.,0]],cell_m=.08))
        g.check(depth(points=[]),(0,0,1),(0,0),('pose',(.01,0,1)),(0,0,1),.1,now=10)
        g.check(dict(status='unscaled'),(0,2,1),(0,0),('pose',(.01,2,1)),(0,2,1),.1,now=60)
        with self.assertRaises(ValueError):  # remembered wall in the path still stops it
            g.check(dict(status='unscaled'),(0,0,1),(0,0),('pose',(.9,0,1)),(0,0,1),.1,now=60)
        with self.assertRaises(ValueError):  # start checks (no clock) still need a fresh frame
            g.check(dict(status='unscaled'),(0,2,1),(0,0),('pose',(.01,2,1)),(0,2,1),.1)

    def test_single_frame_speckle_is_not_remembered_but_repeated_cells_are(self):
        g=ObstacleGuard(dict(clearance_m=.3))
        g.check(dict(depth(points=[[5.,5.]]),geometry_version='a'),(0,0,1),(0,0),('pose',(.01,0,1)),(0,0,1),.1)
        g.check(dict(depth(points=[[9.,9.]]),geometry_version='b'),(0,0,1),(0,0),('pose',(.01,0,1)),(0,0,1),.1)
        self.assertEqual(g.cells,{})
        g.check(dict(depth(points=[[9.08,9.]]),geometry_version='c'),(0,0,1),(0,0),('pose',(.01,0,1)),(0,0,1),.1)
        self.assertEqual(list(g.cells.values()),[(9.08,9.)])
        g.check(dict(depth(points=[[9.08,9.]]),geometry_version='c'),(0,0,1),(0,0),('pose',(.01,0,1)),(0,0,1),.1)
        self.assertEqual(len(g.cells),1)

    def test_inside_clearance_allows_hold_and_moving_away_never_closer(self):
        g=ObstacleGuard(dict(clearance_m=.3,walls=[[.3,0]],cell_m=.08))
        g.check(depth(points=[]),(0,0,1),(0,0),('pose',(0,0,1)),(0,0,1),.1)
        g.check(depth(points=[]),(0,0,1),(0,0),('pose',(-.02,0,1)),(0,0,1),.1)
        with self.assertRaises(ValueError):
            g.check(depth(points=[]),(0,0,1),(0,0),('pose',(.03,0,1)),(0,0,1),.1)
        with self.assertRaises(ValueError):  # measured drift toward the wall
            g.check(depth(points=[]),(0,0,1),(.3,0),('pose',(0,0,1)),(0,0,1),.1)


class RouteAheadTests(unittest.TestCase):
    def test_route_that_bends_away_is_not_a_collision_but_one_into_the_wall_is(self):
        from types import SimpleNamespace
        wall=[[.8,y/10] for y in range(-3,4)]
        g=ObstacleGuard(dict(clearance_m=.3,walls=wall,cell_m=.08))
        bend=SimpleNamespace(elapsed=0.,times=[0.,1.,2.],sample=lambda t:[min(t,.4),max(0.,t-.4)*1.,1.])
        straight=SimpleNamespace(elapsed=0.,times=[0.,2.],sample=lambda t:[t,0.,1.])
        args=(depth(points=[]),(0,0,1),(0,0),('twist',(1.,0,0)),(0,0,1),.1)
        with self.assertRaises(ValueError):g.check(*args)   # straight-line sweep alone trips
        from course_safety import route_ahead
        g.check(*args,route=route_ahead(bend,(0,0),1.))
        with self.assertRaises(ValueError):g.check(*args,route=route_ahead(straight,(0,0),1.))


class LandAfterCourseTests(unittest.TestCase):
    def test_completed_course_lands_and_stopped_course_holds(self):
        import tempfile
        from pathlib import Path
        for controller in ('smooth_position','velocity_tracker'):
            with self.subTest(controller=controller):
                temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
                state=CourseState(Path(temp.name)/'course.json')
                self.assertTrue(state.import_route([[0,0,1],[1,0,1]])[0])
                self.assertTrue(state.capture_anchor([0,0,0],0)[0])
                state.update_preview(depth(points=[]),now=10)
                path=state.execution_path(10)
                c=DashCore();c.owner='pilot';c.owner_hb=10;c.home=(0,0,0,0)
                c.mode='engaged';c.was_offboard=True;c.cmd=[0,0,1];c.target=list(c.cmd)
                c.heading.reset(0);c.last_step=10;c.course_controller=controller
                p=Pos(10,0,0,1,0,True,0,0,0);f=Fc(10,True,True,True,False)
                self.assertTrue(c.start_course(10,path,6,p,f)[0])
                for tick in range(1,600):
                    now=10+.1*tick;p.t=f.t=now;c.heartbeat(now,'pilot')
                    old=(p.x,p.y,p.z);out=c.step(now,p,f,depth(points=[]))
                    if out and out[0]=='pose':p.x,p.y,p.z=out[1];p.yaw=c.heading.yaw
                    elif out:p.x+=out[1][0]*.1;p.y+=out[1][1]*.1;p.z+=out[1][2]*.1;p.yaw+=c.course_yaw_rate*.1
                    p.vx,p.vy,p.vz=((v-a)/.1 for a,v in zip(old,(p.x,p.y,p.z)))
                    if not c.course_run.active:break
                self.assertEqual(c.course_run.state,'complete',c.course_run.status())
                self.assertEqual(c.mode,'landing')
                self.assertAlmostEqual(c.target[2],c._floor())
        c=DashCore();c.mode='engaged';run=SimpleRun('stopped')
        c._land_if_complete(0,run);self.assertEqual(c.mode,'engaged')


class HoldStartHeadingTests(unittest.TestCase):
    def test_fixed_heading_translates_without_turning(self):
        from heading_control import FacingTravel
        h=FacingTravel();h.reset(.7);h.fixed=True
        for _ in range(20):
            self.assertTrue(h.advance_allowed(0,1,.7,0.,.0,.1,continuous=True))  # travel 90 deg left
        self.assertAlmostEqual(h.yaw,.7)
        self.assertEqual(h.status()['policy'],'hold_start')
        self.assertFalse(h.advance_allowed(0,1,.7,0.,.5,.1,continuous=True))  # still waits for lag


class CourseOptionsTests(unittest.TestCase):
    def test_options_change_only_grounded_and_validate_values(self):
        c=DashCore();p=Pos(10,0,0,0,0,True,0,0,0);f=Fc(10,True,False,True,True)
        self.assertTrue(c.select_course_options(10,p,f,obstacle_mode='shadow',heading_mode='hold_start')[0])
        self.assertEqual((c.p.obstacle_mode,c.p.heading_mode),('shadow','hold_start'))
        self.assertTrue(c.heading.fixed)
        self.assertTrue(c.select_course_options(10,p,f,obstacle_mode='avoid')[0])
        self.assertEqual((c.p.obstacle_mode,c.p.heading_mode),('avoid','hold_start'))
        self.assertTrue(c.select_course_options(10,p,f,heading_mode='path_tangent')[0])
        self.assertFalse(c.heading.fixed)
        for bad in (dict(obstacle_mode='x'),dict(heading_mode='spin'),{}):
            self.assertFalse(c.select_course_options(10,p,f,**bad)[0])
        f.armed=True
        self.assertFalse(c.select_course_options(10,p,f,obstacle_mode='shadow')[0])
        self.assertEqual(c.p.obstacle_mode,'avoid')


class NudgeTests(unittest.TestCase):
    def test_wall_ahead_requests_nudge_and_apply_swaps_route_without_stopping(self):
        import tempfile
        from pathlib import Path
        from course_replan import plan_remaining
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        state=CourseState(Path(temp.name)/'course.json')
        self.assertTrue(state.import_route([[1,0,1],[6,0,1]])[0])
        self.assertTrue(state.capture_anchor([0,0,0],0)[0])
        state.update_preview(depth(points=[]),now=10)
        c=DashCore();c.owner='pilot';c.owner_hb=10;c.home=(0,0,0,0)
        c.mode='engaged';c.was_offboard=True;c.cmd=[1.,0,1];c.target=list(c.cmd)
        c.heading.reset(0);c.last_step=10
        p=Pos(10,1.,0,1,0,True,0,0,0);f=Fc(10,True,True,True,False)
        path=state.execution_path(10)
        self.assertTrue(c.start_course(10,path,8,p,f)[0])
        wall=[[2.6,y/10] for y in range(-4,5)]
        c.step(10.1,p,f,depth(points=wall))
        self.assertIsNone(c.paused_course)                 # not stopped
        self.assertIsNotNone(c.nudge_request)              # but a nudge is requested
        run=c.course_run;g=run.obstacle_guard
        nudged=plan_remaining(list(run.points),list(run.times),run.elapsed,tuple(c.cmd),run.anchor,run.reference,
                              run.radius,depth(points=wall),g.known_walls(),known_cell_m=g.cell_m,clearance_m=g.clearance)
        self.assertEqual(nudged['source'],'adjusted')
        c.nudge_request.update(path=nudged,planned_at=10.15)
        self.assertTrue(c.apply_nudge(10.2,nudged)[0])
        self.assertIsNot(c.course_run,run);self.assertTrue(c.course_run.active)
        self.assertEqual(c.mode,'engaged');self.assertIsNone(c.paused_course)
        self.assertGreaterEqual(min(segment_clearance(a,b,wall,.08) for a,b in
                                    zip(c.course_run.points,c.course_run.points[1:])),.4-1e-9)

    def test_route_that_arcs_around_a_far_wall_does_not_nudge(self):
        from types import SimpleNamespace
        g=ObstacleGuard(dict(clearance_m=.3,walls=[[2.3,y/10] for y in range(-5,30)],cell_m=.08))
        arc=SimpleNamespace(elapsed=0.,times=[0.,1.2,3.7],sample=lambda t:[min(t,1.2),max(0.,t-1.2),1.])
        from course_safety import route_ahead
        self.assertFalse(g.route_conflict(route_ahead(arc,(0,0),1.,margin_m=2.),.35))


class TrackingErrorTests(unittest.TestCase):
    def test_velocity_tracker_error_is_distance_to_nearby_route_not_frozen_reference(self):
        from types import SimpleNamespace
        c=DashCore()
        run=SimpleNamespace(controller='velocity_tracker',reference_position=[0.,0.,1.],elapsed=0.,times=[0.,6.],
                            sample=lambda t:[t,0.,1.])
        on_route_ahead=Pos(10,.8,0.,1.,0,True,0,0,0)        # 0.8 m along-track lag
        self.assertLess(c._tracking_error(run,on_route_ahead),.05)
        off_route=Pos(10,.8,.7,1.,0,True,0,0,0)             # 0.7 m cross-track
        self.assertGreater(c._tracking_error(run,off_route),.5)

    def test_nudged_velocity_tracker_never_freezes_to_turn(self):
        import alternate_velocity as av
        run=object.__new__(av.AlternateVelocityRun)
        run.continuous=True;run.min_pace=.6;run._heading_blocked=False
        # The two continuous-mode rules, as written in step():
        error=math.radians(40)
        blocked=abs(error)>=av.EXIT_HEADING and not run.continuous
        self.assertFalse(blocked)
        u=1.;pace=max(1.-u*u*(3.-2.*u),run.min_pace)
        self.assertEqual(pace,.6)


class SimpleRun:
    def __init__(self,state):self.state=state


if __name__=='__main__':unittest.main()
