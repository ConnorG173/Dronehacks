"""CPU-only pairing tests; importing worker does not load Torch/model or start ROS."""
import unittest
from worker import paired_images


class PairTests(unittest.TestCase):
    def test_newest_unmatched_left_uses_previous_complete_pair(self):
        left=[(1_000_000_000,'left1'),(1_033_000_000,'left2')]
        right=[(1_000_001_000,'right1')]
        self.assertEqual(paired_images(left,right,0,1_050_000_000),
                         (1_000_000_000,'left1','right1'))

    def test_stale_future_repeated_and_unsynchronized_pairs_are_rejected(self):
        left=[(1_000_000_000,'left')];right=[(1_000_000_000,'right')]
        self.assertIsNone(paired_images(left,right,0,1_400_000_000))
        self.assertIsNone(paired_images(left,right,0,900_000_000))
        self.assertIsNone(paired_images(left,right,1_000_000_000,1_010_000_000))
        self.assertIsNone(paired_images(left,[(1_010_000_000,'right')],0,1_020_000_000))


if __name__=='__main__':unittest.main()
