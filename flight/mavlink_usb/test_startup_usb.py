"""Synthetic startup gate tests; no ROS node or flight command is executed."""
import math
from types import SimpleNamespace as NS
import unittest

from select_offboard_usb import MAX_AGE, StartupSelector, eligible, mode_command


def samples(now):
    return {
        'status': (now, NS(arming_state=1, failsafe=False, nav_state=2)),
        'land': (now, NS(landed=True)),
        'position': (now, NS(x=1., y=2., z=-.2, xy_valid=True, z_valid=True)),
        'control': (now, NS(position=True, velocity=False, acceleration=False,
                            attitude=False, body_rate=False)),
    }


class StartupTests(unittest.TestCase):
    def test_freshness_missing_future_and_nonfinite_data(self):
        self.assertTrue(eligible(10, samples(10)))
        for key, age in MAX_AGE.items():
            with self.subTest(key=key):
                s = samples(10)
                del s[key]
                self.assertFalse(eligible(10, s))
                s = samples(10)
                s[key] = (10-age-.01, s[key][1])
                self.assertFalse(eligible(10, s))
                s[key] = (10.01, s[key][1])
                self.assertFalse(eligible(10, s))
        for value in (math.nan, math.inf):
            s = samples(10)
            s['position'][1].x = value
            self.assertFalse(eligible(10, s))

    def test_ground_position_failsafe_and_control_gates(self):
        for key, field, value in (
            ('status', 'arming_state', 2), ('status', 'failsafe', True),
            ('land', 'landed', False), ('position', 'xy_valid', False),
            ('position', 'z_valid', False), ('control', 'position', False),
            ('control', 'velocity', True), ('control', 'attitude', True),
        ):
            s = samples(10)
            setattr(s[key][1], field, value)
            self.assertFalse(eligible(10, s))

    def test_two_seconds_then_one_attempt_only(self):
        gate = StartupSelector(10)
        for t in (10, 10.5, 11, 11.9):
            self.assertIsNone(gate.step(t, samples(t), True))
        self.assertEqual(gate.step(12, samples(12), True), 'send')
        for t in (12.1, 13, 15, 16.9):
            self.assertIsNone(gate.step(t, samples(t), True))
        self.assertEqual(gate.step(17, samples(17), True), 'timeout')
        self.assertEqual(gate.step(20, samples(20), True), 'timeout')

    def test_stability_resets_on_outage_or_missing_receiver(self):
        gate = StartupSelector(10)
        self.assertIsNone(gate.step(10, samples(10), True))
        self.assertIsNone(gate.step(11, samples(10), True))
        self.assertIsNone(gate.step(12, samples(12), True))
        self.assertIsNone(gate.step(14, samples(14), False))
        self.assertIsNone(gate.step(15, samples(15), True))
        self.assertEqual(gate.step(17, samples(17), True), 'send')

    def test_armed_airborne_and_already_offboard_never_send(self):
        for key, field, value, expected in (
            ('status', 'arming_state', 2, 'abort'),
            ('land', 'landed', False, 'abort'),
            ('status', 'nav_state', 14, 'confirmed'),
        ):
            gate = StartupSelector(10)
            s = samples(10)
            setattr(s[key][1], field, value)
            self.assertEqual(gate.step(10, s, True), expected)
            self.assertEqual(gate.step(12, samples(12), True), expected)

    def test_deadline_without_telemetry(self):
        gate = StartupSelector(10)
        self.assertIsNone(gate.step(99.9, {}, True))
        self.assertEqual(gate.step(100, {}, True), 'timeout')

    def test_command_is_mode_only(self):
        m = mode_command(12345678)
        self.assertEqual((m.command, m.param1, m.param2), (176, 1., 6.))
        self.assertEqual((m.source_system, m.source_component), (12, 198))
        self.assertEqual((m.target_system, m.target_component), (12, 1))
        self.assertTrue(m.from_external)
        self.assertEqual(m.timestamp, 12345678)


if __name__ == '__main__':
    unittest.main()
