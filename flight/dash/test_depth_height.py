"""Route-height filtering of depth cells; synthetic geometry, no ROS."""
import unittest
from depth_height import filter_for_route


def frame(cells):
    return dict(status='live', age_s=.1, cell_m=.08, points=[[x, y] for x, y, lo, hi in cells],
                z_ranges=[[lo, hi] for x, y, lo, hi in cells])


ROUTE = [[x/10, 0., .76] for x in range(0, 61)]       # straight at 0.76 m (the C2a window height)


class RouteHeightTests(unittest.TestCase):
    def test_window_sill_and_header_drop_out_jambs_stay(self):
        cells = [(3., 0., .0, .30), (3., 0., 1.22, 1.6),          # sill and header at the route
                 (3., -.50, .0, 1.6), (3., .50, .0, 1.6)]          # jambs, full height
        out = filter_for_route(frame(cells), ROUTE)
        self.assertEqual(sorted(map(tuple, out['points'])), [(3., -.5), (3., .5)])
        self.assertEqual(out['height_filtered'], 2)

    def test_tube_roof_and_floor_drop_out_side_walls_stay(self):
        cells = [(2., 0., .0, .06), (2., 0., 1.82, 1.9), (2., -.73, .5, 1.2), (2., .73, .5, 1.2)]
        out = filter_for_route(frame(cells), [[x/10, 0., .94] for x in range(0, 41)])
        self.assertEqual(sorted(map(tuple, out['points'])), [(2., -.73), (2., .73)])

    def test_beam_above_and_box_below_ignored_box_on_route_kept(self):
        cells = [(1., 0., 1.30, 1.45), (2., 0., .0, .40), (4., 0., .60, .90)]
        out = filter_for_route(frame(cells), ROUTE)
        self.assertEqual(out['points'], [[4., 0.]])

    def test_climbing_route_uses_its_own_height_near_each_cell(self):
        climb = [[x/10, 0., .5+x/40] for x in range(0, 41)]   # 0.5 m at x=0 to 1.5 m at x=4
        cells = [(.2, 0., .45, .6), (.2, 0., 1.4, 1.5), (3.8, 0., 1.4, 1.5)]
        out = filter_for_route(frame(cells), climb)
        self.assertEqual(sorted(map(tuple, out['points'])), [(.2, 0.), (3.8, 0.)])

    def test_frames_without_height_or_route_pass_through(self):
        legacy = dict(status='live', age_s=.1, cell_m=.08, points=[[1., 0.]])
        self.assertIs(filter_for_route(legacy, ROUTE), legacy)
        f = frame([(1., 0., 2., 3.)])
        self.assertIs(filter_for_route(f, None), f)
        self.assertIs(filter_for_route(f, []), f)
        stale = dict(f, status='stale')
        self.assertIs(filter_for_route(stale, ROUTE), stale)


if __name__ == '__main__':
    unittest.main()
