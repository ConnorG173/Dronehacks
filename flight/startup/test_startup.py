import copy
import unittest
from types import SimpleNamespace as S

from select_offboard import eligible


class StartupModeTests(unittest.TestCase):
    def setUp(self):
        self.samples = {
            'status': (10, S(arming_state=1)),
            'land': (10, S(landed=True)),
            'flags': (10, S(local_position_invalid=False,
                            local_altitude_invalid=False,
                            offboard_control_signal_lost=False)),
            'control': (10, S(position=True)),
        }

    def test_fresh_disarmed_landed_with_valid_position(self):
        self.assertTrue(eligible(10.1, self.samples))

    def test_missing_or_stale_input_never_enables_mode_change(self):
        for key in self.samples:
            with self.subTest(key=key):
                missing = self.samples.copy()
                del missing[key]
                self.assertFalse(eligible(10.1, missing))
                stale = self.samples.copy()
                stale[key] = (9, stale[key][1])
                self.assertFalse(eligible(10.1, stale))

    def test_armed_airborne_or_invalid_estimate_blocks_mode_change(self):
        for key, attr, value in (
            ('status', 'arming_state', 2), ('land', 'landed', False),
            ('flags', 'local_position_invalid', True),
            ('flags', 'local_altitude_invalid', True),
            ('flags', 'offboard_control_signal_lost', True),
            ('control', 'position', False),
        ):
            with self.subTest(attr=attr):
                samples = copy.deepcopy(self.samples)
                setattr(samples[key][1], attr, value)
                self.assertFalse(eligible(10.1, samples))


if __name__ == '__main__':
    unittest.main()
