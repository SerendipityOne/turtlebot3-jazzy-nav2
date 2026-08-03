#!/usr/bin/env python3
"""Behavioral checks for the raw VLA dataset gate."""

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).with_name('validate_raw_vla_dataset.py')
SPEC = importlib.util.spec_from_file_location('validate_raw_vla_dataset', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


def write_episode(
        root: Path, *, target_present: bool, action_id: int, scan_progress: float,
        target_visible: bool, target_label_pixels: int, coverage_complete: bool) -> None:
    episode = root / 'episodes' / 'episode_0'
    (episode / 'frames').mkdir(parents=True)
    (episode / 'frames' / '0.jpg').touch()
    (episode / 'episode.json').write_text(json.dumps({
        'episode_id': 0, 'seed': 1, 'split': 'train',
        'target_color': 'red', 'target_present': target_present,
    }), encoding='utf-8')
    state = [0.0] * 23
    state[22] = scan_progress
    (episode / 'steps.jsonl').write_text(json.dumps({
        'image': 'frames/0.jpg', 'instruction': '找红球', 'robot_state': state,
        'action_id': action_id,
        'oracle': {
            'target_visible': target_visible,
            'target_label_pixels': target_label_pixels,
            'coverage_complete': coverage_complete,
        },
    }) + '\n', encoding='utf-8')


class ValidateRawVlaDatasetTest(unittest.TestCase):
    def run_validator(self, **episode_kwargs: object) -> tuple[int, dict]:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'raw'
            report_dir = Path(temporary) / 'report'
            write_episode(root, **episode_kwargs)
            with patch.object(sys, 'argv', [str(SCRIPT), '--raw-root', str(root),
                                            '--report-dir', str(report_dir)]):
                result = VALIDATOR.main()
            return result, json.loads((report_dir / 'report.json').read_text(encoding='utf-8'))

    def test_accepts_completed_target_absent_report(self) -> None:
        result, report = self.run_validator(
            target_present=False, action_id=9, scan_progress=1.0,
            target_visible=False, target_label_pixels=0, coverage_complete=True)
        self.assertEqual(result, 0)
        self.assertTrue(report['valid'])
        self.assertEqual(report['semantic_counts']['target_absent_reported'], 1)

    def test_accepts_visible_matching_target_approach(self) -> None:
        result, report = self.run_validator(
            target_present=True, action_id=5, scan_progress=0.0,
            target_visible=True, target_label_pixels=1, coverage_complete=False)
        self.assertEqual(result, 0)
        self.assertTrue(report['valid'])
        self.assertEqual(report['semantic_counts']['target_present_approached'], 1)

    def test_rejects_invisible_target_approach(self) -> None:
        result, report = self.run_validator(
            target_present=True, action_id=5, scan_progress=0.0,
            target_visible=False, target_label_pixels=0, coverage_complete=False)
        self.assertEqual(result, 1)
        self.assertFalse(report['valid'])
        self.assertIn('target-visibility contract', report['issues'][0])

    def test_rejects_target_present_report_not_found(self) -> None:
        result, report = self.run_validator(
            target_present=True, action_id=9, scan_progress=1.0,
            target_visible=False, target_label_pixels=0, coverage_complete=True)
        self.assertEqual(result, 1)
        self.assertFalse(report['valid'])
        self.assertEqual(report['semantic_counts']['target_present_missed'], 1)

    def test_rejects_candidate_before_scan_completion(self) -> None:
        result, report = self.run_validator(
            target_present=False, action_id=0, scan_progress=0.0,
            target_visible=False, target_label_pixels=0, coverage_complete=False)
        self.assertEqual(result, 1)
        self.assertFalse(report['valid'])

    def test_manifest_changes_when_oracle_labels_change(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'raw'
            write_episode(
                root, target_present=False, action_id=9, scan_progress=1.0,
                target_visible=False, target_label_pixels=0, coverage_complete=True)
            before = VALIDATOR.raw_manifest_sha256(root)
            steps = root / 'episodes' / 'episode_0' / 'steps.jsonl'
            steps.write_text(steps.read_text(encoding='utf-8') + '\n', encoding='utf-8')
            self.assertNotEqual(before, VALIDATOR.raw_manifest_sha256(root))


if __name__ == '__main__':
    unittest.main()
