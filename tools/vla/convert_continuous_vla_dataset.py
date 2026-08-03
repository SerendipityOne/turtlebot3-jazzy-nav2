#!/usr/bin/env python3
"""Convert approved continuous-local-goal Oracle episodes to a local LeRobot dataset."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from validate_continuous_vla_dataset import (
    ACTION_SIZE,
    REPORT_SCHEMA_VERSION,
    STATE_SIZE,
    raw_manifest_sha256,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', required=True, type=Path)
    parser.add_argument('--validation-report', required=True, type=Path)
    parser.add_argument('--split', required=True, choices=('train', 'validation', 'test'))
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--overwrite', action='store_true')
    return parser.parse_args()


def require_report(raw_root: Path, report_path: Path) -> dict[str, Any]:
    try:
        report = json.loads(report_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise SystemExit(f'invalid validation report: {error}') from error
    if (report.get('schema_version') != REPORT_SCHEMA_VERSION or
            report.get('policy_interface') != 'continuous_local_goal' or
            report.get('valid') is not True):
        raise SystemExit('validation report does not approve continuous local-goal conversion')
    if Path(report.get('raw_root', '')).resolve() != raw_root.resolve():
        raise SystemExit('validation report raw_root does not match --raw-root')
    if report.get('raw_manifest_sha256') != raw_manifest_sha256(raw_root):
        raise SystemExit('raw dataset changed after validation; generate a new validation report')
    return report


def features(height: int, width: int) -> dict[str, dict[str, Any]]:
    return {
        'observation.images.front': {
            'dtype': 'image', 'shape': (height, width, 3),
            'names': ['height', 'width', 'channels'],
        },
        'observation.state': {
            'dtype': 'float32', 'shape': (STATE_SIZE,),
            'names': [f'state_{index}' for index in range(STATE_SIZE)],
        },
        'action': {
            'dtype': 'float32', 'shape': (ACTION_SIZE,),
            'names': ['delta_x_m', 'delta_y_m', 'delta_yaw_rad'],
        },
    }


def rows(episode_dir: Path) -> Iterable[dict[str, Any]]:
    for line_number, line in enumerate((episode_dir / 'steps.jsonl').read_text(encoding='utf-8').splitlines(), 1):
        if line.strip():
            row = json.loads(line)
            row['_line'] = line_number
            yield row


def frame(row: dict[str, Any], episode_dir: Path) -> tuple[Path, np.ndarray, np.ndarray, str]:
    image_path = episode_dir / row['image']
    state = np.asarray(row['robot_state'], dtype=np.float32)
    action = np.asarray(row['action'], dtype=np.float32)
    instruction = row['instruction']
    if not image_path.is_file() or state.shape != (STATE_SIZE,) or action.shape != (ACTION_SIZE,):
        raise ValueError(f'invalid approved frame at {episode_dir}:{row["_line"]}')
    if not np.isfinite(state).all() or not np.isfinite(action).all() or not instruction.strip():
        raise ValueError(f'non-finite or empty approved frame at {episode_dir}:{row["_line"]}')
    return image_path, state, action, instruction


def main() -> None:
    args = parse_args()
    raw_root = args.raw_root.resolve()
    report = require_report(raw_root, args.validation_report)
    selected = [
        directory for directory in sorted((raw_root / 'episodes').glob('*')) if directory.is_dir() and
        json.loads((directory / 'episode.json').read_text(encoding='utf-8')).get('split') == args.split
    ]
    if not selected:
        raise SystemExit(f'no {args.split} episodes found')
    if args.output_root.exists():
        if not args.overwrite:
            raise SystemExit(f'output already exists: {args.output_root}; pass --overwrite after review')
        shutil.rmtree(args.output_root)

    first = frame(next(rows(selected[0])), selected[0])
    with Image.open(first[0]) as image:
        width, height = image.size
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id, root=args.output_root, fps=1,
        robot_type='turtlebot3_waffle_pi_cam', features=features(height, width),
        use_videos=False, image_writer_threads=4)
    count = 0
    try:
        for episode_dir in selected:
            for row in rows(episode_dir):
                image_path, state, action, instruction = frame(row, episode_dir)
                with Image.open(image_path) as image:
                    dataset.add_frame({
                        'observation.images.front': np.asarray(image.convert('RGB')),
                        'observation.state': state, 'action': action, 'task': instruction,
                    })
                count += 1
            dataset.save_episode()
    finally:
        dataset.finalize()
    (args.output_root / 'conversion_report.json').write_text(json.dumps({
        'policy_interface': 'continuous_local_goal',
        'validation_manifest_sha256': report['raw_manifest_sha256'],
        'split': args.split, 'episodes': len(selected), 'frames': count,
    }, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
