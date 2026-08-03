#!/usr/bin/env python3
"""Behavioral checks for train-only bounded action weighting."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name('action_balanced_weighting.py')
SPEC = importlib.util.spec_from_file_location('action_balanced_weighting', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
WEIGHTING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WEIGHTING)


class ActionBalancedWeightingTest(unittest.TestCase):
    def test_weights_are_bounded_and_mean_one(self) -> None:
        counts = {str(action_id): 100 for action_id in range(10)}
        counts['9'] = 1
        weights = WEIGHTING.action_weights_from_counts(counts)
        weighted_mean = sum(counts[str(action_id)] * weights[action_id] for action_id in range(10)) / sum(counts.values())
        self.assertAlmostEqual(weighted_mean, 1.0)
        self.assertLessEqual(max(weights) / min(weights), 4.0)
        self.assertGreater(weights[9], weights[0])

    def test_rejects_missing_action_class(self) -> None:
        with self.assertRaisesRegex(ValueError, 'action_counts\\[9\\]'):
            WEIGHTING.action_weights_from_counts({str(action_id): 1 for action_id in range(9)})

    def test_load_rejects_non_train_split(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'conversion_report.json').write_text(json.dumps({
                'split': 'validation',
                'action_counts': {str(action_id): 1 for action_id in range(10)},
            }), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'only for a train split'):
                WEIGHTING.load_train_action_weights(root)


if __name__ == '__main__':
    unittest.main()
