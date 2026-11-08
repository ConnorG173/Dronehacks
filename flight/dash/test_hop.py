"""Course + return hop: course, PX4 Land, automatic takeoff into the hop, PX4 Land.

Closed loop through the real DashCore / DashNode doubles: the simulated drone goes where it is
commanded (pose) and follows the commanded heading; a simulated PX4 lands it when the node sends
Land mode (DO_SET_MODE 1, 4, 6) and then (optionally) auto-disarms on the ground.
"""
import unittest

from dash_core import LANDING, ENGAGED, IDLE
import test_autoflight_api_offline as api


def hop_path(n, top=2.3):
    """Return hop in the course frame: from the course end (4, 0) over a 'wall' back to (0, 0)."""
    p = n.course.original_path()
    return dict(p, points=[(4., 0., 1.), (3., 0., top), (1., 0., top), (0., 0., 1.)], times_s=None,
                avoidance=None, marks=None)


def land_requests(n):
    return [c.args[0] for c in n.arm_pub.publish.call_args_list
            if c.args[0].command == 176 and (c.args[0].param1, c.args[0].param2, getattr(c.args[0], 'param3', 0.)) == (1., 4., 6.)]


class HopTests(unittest.TestCase):
    node = api.FlightApiTests.node

    def setup(self, top=2.3, max_alt=2.5, z_bias=0., px4_lands=True, px4_disarms=True):
        n = self.node()
        n.core.p.obstacle_mode = 'shadow'
        n.core.p.max_alt_m = max_alt
        hop = hop_path(n, top)
        n.course.return_path = lambda: hop
        self.z_bias = z_bias            # altitude estimate error: the reported z is off by this much
        self.px4_lands = px4_lands      # False: PX4 ignores the Land request
        self.px4_disarms = px4_disarms  # False: PX4 stays armed on the ground after Land
        return n

    def fly(self, n, seconds=120., until=None):
        """Step the core at 10 Hz with the node's PX4 Land relay. Returns the trace."""
        core, pos, fc = n.core, n.pos, n.fc
        trace, t, true_z, in_land = [], pos.t, pos.z - self.z_bias, False
        t0 = t
        while t < t0 + seconds:
            t = round(t + .1, 3)
            pos.t = fc.t = t
            n.land = (t, fc.landed)
            core.heartbeat(t, 'pilot')
            req = core.px4_land_request                      # the node's relay (as in _tick)
            if req is not None and req != getattr(n, '_px4_land_sent', None):
                n._px4_land_sent = req
                n.request_px4_land(t)
                in_land = self.px4_lands
            if in_land:                                      # PX4 Land mode: descend, then (auto-)disarm
                fc.offboard_active = False
                true_z = max(0., true_z - .05)
                if true_z == 0.:
                    fc.landed, in_land = True, False
                    fc.armed = not self.px4_disarms
                out = core.step(t, pos, fc)
            else:
                out = core.step(t, pos, fc)
                if out is not None and out[0] == 'pose' and fc.offboard_active:
                    pos.x, pos.y = out[1][0], out[1][1]
                    true_z = out[1][2] - self.z_bias
                if fc.armed and fc.offboard_active:
                    fc.landed = true_z < .02
            pos.vx = pos.vy = pos.vz = 0.
            pos.z = true_z + self.z_bias
            if core.heading.yaw is not None and fc.armed:
                pos.yaw = core.heading.yaw                   # the drone turns where it is told
            trace.append((t, core.mode, None if core.hop is None else core.hop['state'], out, pos.z))
            if until and until(core):
                break
        return trace

    def airborne(self, n, z=1.):
        n.fc.armed, n.fc.landed, n.fc.offboard_active = True, False, True
        n.core.was_offboard = True
        n.pos.z = z + self.z_bias
        n.core.cmd = [n.pos.x, n.pos.y, z]

    def launch(self, n):
        ok, msg = n.do_command(n.pos.t, 'pilot', dict(op='launch', flight='course', radius_m=6., with_return=True))
        self.assertTrue(ok, msg)
        self.assertEqual(n.core.hop['state'], 'armed')
        self.airborne(n)                                     # takeoff done (as test_shadow_mode does)

    def auto_takeoff(self, n):
        """The node's automatic hop takeoff, then PX4 arming / Offboard confirmed."""
        n.land = (n.pos.t, True)                             # land detector: on the ground
        n._hop_auto = dict(next=0., tries=0)
        api.DashNode._auto_hop_takeoff(n, n.pos.t)
        self.assertEqual(n.core.hop['state'], 'flying', n.core.events[-1])
        self.airborne(n, z=0.)
        n.fc.landed = True

    def test_course_px4_land_auto_takeoff_hop_px4_land(self):
        n = self.setup()
        self.launch(n)
        self.fly(n, until=lambda c: c.hop is not None and c.hop['state'] == 'ready')
        self.assertEqual(n.core.mode, IDLE, n.core.reason)
        self.assertEqual(len(land_requests(n)), 1)           # exactly one PX4 Land request
        self.auto_takeoff(n)
        trace = self.fly(n, until=lambda c: c.hop is None)
        self.assertIsNone(n.core.hop, n.core.reason)
        self.assertEqual(len(land_requests(n)), 2)           # PX4 Land at the hop end too
        hop_z = [z for _, _, s, out, z in trace if s == 'flying' and out and out[0] == 'pose']
        self.assertGreater(max(hop_z), 2.25)                 # above the 2.0 m course ceiling
        self.assertLess(abs(n.pos.x) + abs(n.pos.y), .2)     # back at the start
        self.assertFalse(any(m == LANDING for _, m, _, _, _ in trace))   # never the dashboard landing

    def test_still_armed_after_land_takes_off_without_rearming(self):
        n = self.setup(px4_disarms=False)
        self.launch(n)
        self.fly(n, until=lambda c: c.hop is not None and c.hop['state'] == 'ready')
        n.arm_pub.publish.reset_mock()
        n.core.owner_hb = n.pos.t - 99.                      # works without the laptop
        n._hop_auto = dict(next=0., tries=0)
        api.DashNode._auto_hop_takeoff(n, n.pos.t)
        self.assertEqual(n.core.hop['state'], 'flying', n.core.events[-1])
        sent = [(m.command, m.param1, m.param2) for m in (c.args[0] for c in n.arm_pub.publish.call_args_list)]
        self.assertEqual(sent, [(176, 1., 6.)])              # Offboard only: no disarm / arm

    def test_climb_is_not_throttled_to_the_manual_rate(self):
        n = self.setup()
        self.launch(n)
        self.fly(n, until=lambda c: c.hop is not None and c.hop['state'] == 'ready')
        self.auto_takeoff(n)
        trace = self.fly(n, until=lambda c: c.hop is None)
        flying = [t for t, _, s, _, _ in trace if s == 'flying']
        self.assertLess(flying[-1] - flying[0], 12.)         # 0.3 m/s climbing made this > 15 s

    def test_biased_altitude_still_finishes_the_course(self):
        n = self.setup(z_bias=.3)     # reported z 0.3 m high: the strict 15 cm 3D end check never finishes
        self.launch(n)
        self.fly(n, seconds=60., until=lambda c: c.hop is not None and c.hop['state'] != 'armed')
        self.assertEqual(n.core.hop['state'], 'land', n.core.reason)

    def test_dashboard_lands_if_px4_ignores_land(self):
        n = self.setup(px4_lands=False)
        self.launch(n)
        self.fly(n, until=lambda c: c.mode == LANDING)
        self.assertEqual(n.core.mode, LANDING)
        self.assertEqual(len(land_requests(n)), 1)

    def test_hop_takeoff_refused_far_from_its_start(self):
        n = self.setup()
        self.assertFalse(n.do_command(10., 'pilot', dict(op='launch', flight='hop', radius_m=6.))[0])
        self.launch(n)
        self.fly(n, until=lambda c: c.hop is not None and c.hop['state'] == 'ready')
        n.pos.x += .5                                        # carried 0.5 m away from the hop start
        n.land = (n.pos.t, True)
        ok, msg = n.do_command(n.pos.t, 'pilot', dict(op='launch', flight='hop', radius_m=6.))
        self.assertFalse(ok)
        self.assertIn('return hop start', msg)
        self.assertEqual(n.core.hop['state'], 'ready')

    def test_hop_refused_above_the_interface_limit(self):
        n = self.setup(max_alt=2.0)                    # flight interface still at 2.0 m
        ok, msg = n.do_command(10., 'pilot', dict(op='launch', flight='course', radius_m=6., with_return=True))
        self.assertFalse(ok)
        self.assertIn('Return hop reaches', msg)
        self.assertIsNone(n.core.hop)

    def test_hop_must_start_at_the_course_end(self):
        n = self.setup()
        n.course.return_path()['points'][0] = (3., 1., 1.)
        ok, msg = n.do_command(10., 'pilot', dict(op='launch', flight='course', radius_m=6., with_return=True))
        self.assertFalse(ok)
        self.assertIn('starts', msg)

    def test_normal_course_keeps_the_2m_ceiling_and_lands(self):
        n = self.setup()
        ok, msg = n.do_command(10., 'pilot', dict(op='launch', flight='course', radius_m=6.))
        self.assertTrue(ok, msg)
        self.assertIsNone(n.core.hop)
        self.assertAlmostEqual(n.core._ceiling(), n.core.ground_z + 2.0)
        self.airborne(n)
        self.fly(n, until=lambda c: c.mode == LANDING)
        self.assertEqual(n.core.mode, LANDING)
        self.assertEqual(land_requests(n), [])

    def test_no_holds_keeps_flying_through_laptop_and_tracking_faults(self):
        n = self.setup()
        n.core.p.no_holds = True
        self.launch(n)
        self.fly(n, seconds=1.)
        core, t = n.core, n.pos.t
        self.assertTrue(core.course_run.active)
        core.owner_hb = t - 30.                                   # laptop gone for 30 s
        n.pos.x += 3.                                             # 3 m tracking error, outside the fence
        n.fc.landed = True                                        # spurious landed report
        out = core.step(t + .1, n.pos, n.fc)
        self.assertEqual(core.mode, ENGAGED, core.reason)
        self.assertTrue(core.course_run.active)
        self.assertIsNotNone(out)
        n.pos.valid = False                                       # PX4 position lost: still stops
        core.step(t + .2, n.pos, n.fc)
        self.assertNotEqual(core.mode, ENGAGED)

    def test_stop_cancels_the_hop(self):
        n = self.setup()
        self.launch(n)
        n.core.stop_course(10.)
        self.assertIsNone(n.core.hop)


if __name__ == '__main__':
    unittest.main()
