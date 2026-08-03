#!/usr/bin/env python3
"""Small contract checks for continuous raw-data validation."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from validate_continuous_vla_dataset import validate_step


class ContinuousDataValidationTest(unittest.TestCase):
    def test_accepts_bounded_search_goal_and_rejects_unbounded_goal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode = Path(temporary)
            (episode / 'frame.jpg').write_bytes(b'placeholder')
            state = [0.0] * 25
            state[22] = 0.2
            state[24] = 1.0
            row = {
                'image': 'frame.jpg', 'instruction': 'search for red ball',
                'robot_state': state, 'action': [0.5, 0.0, 0.1],
                'oracle': {'target_visible': False, 'target_label_pixels': 0, 'stage': 'search'},
            }
            issues: list[str] = []
            record = validate_step(episode, 1, row, {'target_present': False}, issues)
            self.assertEqual(record, ('search', False, 0.5))
            self.assertEqual(issues, [])

            row['action'] = [0.8, 0.0, 0.1]
            validate_step(episode, 2, row, {'target_present': False}, issues)
            self.assertTrue(any('exceeds the continuous local-goal bounds' in item for item in issues))


if __name__ == '__main__':
    unittest.main()
