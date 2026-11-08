import math
import unittest

from pass_through import PassThrough, centre_route, find_opening


def wall(hole_w, hole_h, cx=0., cz=1.0, y=3.0, half=1.6):
    """Wall across a +y route with a hole_w x hole_h hole centred (cx, cz); 8 cm cells, 20 cm slabs."""
    cells = []
    for i in range(int(2*half/.08)+1):
        x = round(-half+.08*i, 3)
        zs = [.2+.05*k for k in range(37)]
        if abs(x-cx) < hole_w/2:
            zs = [z for z in zs if abs(z-cz) >= hole_h/2]
        slabs = {}
        for z in zs:
            slabs.setdefault(math.floor(z/.2), []).append(z)
        cells += [[x, y, min(v), max(v)] for v in slabs.values()]
    return cells


ROUTE = [[0., i/10, 1.] for i in range(61)]


class PassThroughTests(unittest.TestCase):
    def test_window_off_centre_is_found_and_centred_both_axes(self):
        o = find_opening(ROUTE, wall(.80, .80, cx=.20, cz=1.15), holes=[[.20, 3.0, 1.15]])
        self.assertIsNotNone(o)
        self.assertAlmostEqual(-o['lat'], .20, delta=.03)    # +x is right of a +y route
        self.assertAlmostEqual(o['dz'], .15, delta=.03)
        self.assertGreater(o['room'], o['room_before'])
        route, found = centre_route(ROUTE, wall(.80, .80, cx=.20, cz=1.15), holes=[[.20, 3.0, 1.15]])
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(route[30][0], .20, delta=.03)
        self.assertAlmostEqual(route[30][2], 1.15, delta=.03)
        self.assertEqual(list(route[0]), ROUTE[0])            # ends stay put
        self.assertEqual(list(route[-1]), ROUTE[-1])

    def test_solid_wall_or_too_tight_is_never_steered_into(self):
        for cells in (wall(0., 0.), wall(.40, .80)):
            o = find_opening(ROUTE, cells, holes=[[0., 3.0, 1.0]])
            if o is not None:
                self.assertEqual((o['lat'], o['dz']), (0., 0.))

    def test_nothing_marked_never_acts(self):
        cells = wall(.80, .80, cx=.20)
        self.assertIsNone(find_opening(ROUTE, cells))
        self.assertIsNone(find_opening(ROUTE, cells, holes=[[3., 3.0, 1.0]]))   # marked elsewhere
        route, found = centre_route(ROUTE, cells)
        self.assertEqual(found, [])
        pt = PassThrough(ROUTE)
        for k in range(5):
            self.assertIsNone(pt.update([0., 1.5, 1.], cells, f'f{k}', .1))

    def test_solid_object_straddling_the_route_is_not_an_opening(self):
        # A box 0.5 m wide right on the line at 0.6-1.6 m height, nothing around it, at a marked hole:
        # it could be flown under or around, but it is not a hole to thread.
        box = [[x/100, 3.0+j*.08, lo, lo+.2] for x in range(-24, 25, 8) for j in range(3)
               for lo in (.6, .8, 1.0, 1.2, 1.4)]
        self.assertIsNone(find_opening(ROUTE, box, holes=[[0., 3.0, 1.0]]))

    def test_open_route_has_no_opening(self):
        self.assertIsNone(find_opening(ROUTE, [[1.5, 3.0, .2, 2.0]]))
        self.assertIsNone(find_opening(ROUTE, []))

    def test_floor_bounds_the_vertical_shift(self):
        o = find_opening(ROUTE, wall(.80, .80, cz=.70), floor=.85, holes=[[0., 3.0, .70]])
        self.assertGreaterEqual(1.0+o['dz'], .85-1e-9)

    def test_hover_until_the_estimate_settles_then_commit_straight_through(self):
        pt = PassThrough(ROUTE, holes=[[.20, 3.0, 1.0]])
        steady = wall(.80, .80, cx=.20)
        self.assertIsNone(pt.update([0., 1.0, 1.], steady, 'a', .1))
        self.assertEqual(pt.state, 'normal')                            # still 2 m out
        pt.update([0., 1.55, 1.], steady, 'b', .1)
        self.assertEqual((pt.state, pt.hold), ('hover', True))         # 1.45 m before: hover, clock stops
        for k in range(8):                                               # estimate still moving: never commit
            pt.update([0., 1.55, 1.], wall(.80, .80, cx=.12 if k % 2 else .28), f'j{k}', .1)
        self.assertEqual(pt.state, 'hover')
        route = None
        for k in range(30):
            r = pt.update([0., 1.55, 1.], steady, f's{k}', .1)
            if r is not None:
                route = r
                break
        self.assertEqual((pt.state, pt.hold), ('commit', False))
        self.assertAlmostEqual(pt.gate[0], .20, delta=.03)              # locked on the measured centre
        self.assertAlmostEqual(pt.gate[2], 1.0, delta=.03)
        at_gate = min(route, key=lambda p: abs(p[1]-3.0))
        self.assertAlmostEqual(at_gate[0], .20, delta=.03)
        self.assertEqual(list(route[0]), ROUTE[0])
        self.assertEqual(list(route[-1]), ROUTE[-1])                   # back on the plan after the exit
        pt.update([0., 5.6, 1.], None, None, .1)
        self.assertEqual(pt.state, 'normal')
        self.assertTrue(pt.holes[0]['done'])

    def test_no_hole_seen_keeps_hovering_and_never_commits(self):
        pt = PassThrough(ROUTE, holes=[[.20, 3.0, 1.0]])
        pt.update([0., 1.55, 1.], [], 'a', .1)
        for k in range(40):
            self.assertIsNone(pt.update([0., 1.55, 1.], [], f'e{k}', .1))
        self.assertEqual((pt.state, pt.hold, pt.gate), ('hover', True, None))
        self.assertIn('waiting', pt.status()['reason'])

    def test_ordinary_room_clutter_is_not_an_opening_to_correct(self):
        # Things 0.6-0.9 m either side and a table below / beam above: plenty of room, no correction.
        cells = [[-.7, 2.0, .2, 1.6], [.65, 2.4, .2, 1.6], [0., 2.2, .0, .5], [.1, 2.2, 1.6, 1.8]]
        o = find_opening(ROUTE, cells)
        self.assertTrue(o is None or not o['needed'])
        pt = PassThrough(ROUTE)
        for k in range(5):
            self.assertIsNone(pt.update([0., 1., 1.], cells, f'f{k}', .1))
        self.assertEqual(pt.state, 'normal')

    def test_centred_window_needs_no_correction_and_reports_its_frame(self):
        o = find_opening(ROUTE, wall(.80, .80), holes=[[0., 3.0, 1.0]])
        self.assertIsNotNone(o)
        self.assertFalse(o['needed'])
        self.assertEqual((o['lat'], o['dz']), (0., 0.))
        for k in ('left_m', 'right_m', 'above_m', 'below_m'):
            self.assertAlmostEqual(o[k], .40, delta=.06)


if __name__ == '__main__':
    unittest.main()
