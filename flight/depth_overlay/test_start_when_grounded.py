"""Pure tests: no ROS imports, worker imports, inference, exec, or flight commands."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from start_when_grounded import GroundedGate, blocked_reason, read_ground_state, worker_path


NOW = 1_800_000_000_000_000_000


def good():
    return dict(wall_ns=NOW, router_alive=True, connected=True,
                fc_armed=False, fc_landed=True,
                fc_heartbeat_age_s=.95, fc_landed_age_s=.15)


class GroundGuardTests(unittest.TestCase):
    def test_good_status_survives_normal_one_second_report_cadence(self):
        self.assertIsNone(blocked_reason(good(), NOW))
        self.assertIsNone(blocked_reason(good(), NOW + 990_000_000))

    def test_unknown_armed_airborne_disconnected_and_malformed_fail_closed(self):
        for key, value in (
            ('router_alive', False), ('connected', False), ('fc_armed', True),
            ('fc_landed', False), ('fc_armed', None), ('fc_landed', None),
            ('wall_ns', '123'), ('wall_ns', True), ('wall_ns', 10**400),
            ('fc_heartbeat_age_s', math.nan), ('fc_landed_age_s', math.inf),
            ('fc_heartbeat_age_s', 10**400),
            ('fc_landed_age_s', -1), ('fc_landed_age_s', True),
        ):
            s = good()
            s[key] = value
            self.assertIsNotNone(blocked_reason(s, NOW), (key, value))
        for key in good():
            s = good()
            del s[key]
            self.assertIsNotNone(blocked_reason(s, NOW), key)
        self.assertIsNotNone(blocked_reason([], NOW))

    def test_effective_message_ages_include_file_age(self):
        s = good()
        s['fc_landed_age_s'] = .7
        self.assertIsNotNone(blocked_reason(s, NOW + 900_000_000))
        s = good()
        s['fc_heartbeat_age_s'] = 1.6
        self.assertIsNotNone(blocked_reason(s, NOW))
        s = good()
        s['fc_landed_age_s'] = 1.1
        self.assertIsNotNone(blocked_reason(s, NOW))
        self.assertIsNotNone(blocked_reason(good(), NOW + 2_000_000_001))
        self.assertIsNotNone(blocked_reason(good(), NOW - 1))

    def test_stability_restarts_after_bad_state_or_clock_rollback(self):
        gate = GroundedGate()
        self.assertFalse(gate.update(10, None))
        self.assertFalse(gate.update(11.99, None))
        self.assertTrue(gate.update(12, None))
        self.assertFalse(gate.update(12.1, 'armed'))
        self.assertFalse(gate.update(13, None))
        self.assertFalse(gate.update(14.9, None))
        self.assertTrue(gate.update(15, None))
        self.assertFalse(gate.update(1, None))
        self.assertFalse(gate.update(2.9, None))
        self.assertTrue(gate.update(3, None))

    def test_file_reader_rejects_missing_partial_and_oversized_json(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'status.json'
            self.assertIsNotNone(read_ground_state(p, NOW))
            p.write_text('{')
            self.assertIsNotNone(read_ground_state(p, NOW))
            p.write_text(' ' * 65537)
            self.assertIsNotNone(read_ground_state(p, NOW))
            p.write_text(json.dumps(good()))
            self.assertIsNone(read_ground_state(p, NOW))

    def test_only_local_regular_worker_file_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            worker = root / 'worker.py'
            with self.assertRaises(FileNotFoundError):
                worker_path(root)
            worker.write_text('')
            self.assertEqual(worker_path(root), worker)
            worker.unlink()
            worker.symlink_to('/bin/sh')
            with self.assertRaises(RuntimeError):
                worker_path(root)


if __name__ == '__main__':
    unittest.main()
