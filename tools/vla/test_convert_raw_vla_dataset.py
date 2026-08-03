#!/usr/bin/env python3
"""Behavioral checks for the conversion validation-report gate."""

import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).parent
SCRIPT = TOOLS_DIR / 'convert_raw_vla_dataset.py'
sys.path.insert(0, str(TOOLS_DIR))
fake_lerobot = types.ModuleType('lerobot')
fake_datasets = types.ModuleType('lerobot.datasets')
fake_dataset = types.ModuleType('lerobot.datasets.lerobot_dataset')
fake_dataset.LeRobotDataset = object
fake_modules = {
    'lerobot': fake_lerobot,
    'lerobot.datasets': fake_datasets,
    'lerobot.datasets.lerobot_dataset': fake_dataset,
}
missing_module = object()
original_modules = {name: sys.modules.get(name, missing_module) for name in fake_modules}
try:
    sys.modules.update(fake_modules)
    SPEC = importlib.util.spec_from_file_location('convert_raw_vla_dataset', SCRIPT)
    assert SPEC is not None and SPEC.loader is not None
    CONVERTER = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(CONVERTER)
finally:
    for name, module in original_modules.items():
        if module is missing_module:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def write_raw_root(root: Path) -> None:
    episode = root / 'episodes' / 'episode_0'
    (episode / 'frames').mkdir(parents=True)
    (episode / 'episode.json').write_text('{}\n', encoding='utf-8')
    (episode / 'steps.jsonl').write_text('{}\n', encoding='utf-8')
    (episode / 'frames' / '0.jpg').write_bytes(b'jpeg')


class ConvertRawVlaDatasetTest(unittest.TestCase):
    def write_report(self, root: Path, report_path: Path, *, valid: bool = True) -> None:
        report_path.write_text(json.dumps({
            'schema_version': CONVERTER.REPORT_SCHEMA_VERSION,
            'valid': valid,
            'raw_root': str(root.resolve()),
            'raw_manifest_sha256': CONVERTER.raw_manifest_sha256(root),
        }), encoding='utf-8')

    def test_accepts_matching_valid_report(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'raw'
            report_path = Path(temporary) / 'report.json'
            write_raw_root(root)
            self.write_report(root, report_path)
            report = CONVERTER.require_validation_report(root, report_path)
            self.assertTrue(report['valid'])

    def test_rejects_report_after_raw_labels_change(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'raw'
            report_path = Path(temporary) / 'report.json'
            write_raw_root(root)
            self.write_report(root, report_path)
            (root / 'episodes' / 'episode_0' / 'steps.jsonl').write_text('{"changed": true}\n', encoding='utf-8')
            with self.assertRaisesRegex(SystemExit, 'changed after validation'):
                CONVERTER.require_validation_report(root, report_path)


if __name__ == '__main__':
    unittest.main()
