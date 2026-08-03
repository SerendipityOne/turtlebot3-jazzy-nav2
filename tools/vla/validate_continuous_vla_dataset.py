#!/usr/bin/env python3
"""Validate continuous-local-goal Gazebo Oracle episodes before LeRobot conversion."""

from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path
from typing import Any

from validate_raw_vla_dataset import raw_manifest_sha256


REPORT_SCHEMA_VERSION = 1
STATE_SIZE = 25
ACTION_SIZE = 3
MAX_TRANSLATION_M = 0.75
MAX_YAW_RAD = math.pi / 4.0
SPLIT_RATIOS = {'train': 0.70, 'validation': 0.15, 'test': 0.15}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', required=True, type=Path)
    parser.add_argument('--report-dir', required=True, type=Path)
    return parser.parse_args()


def issue(issues: list[str], episode: Path, line: int, detail: str) -> None:
    issues.append(f'{episode.name}:step {line}: {detail}')


def finite_vector(value: Any, size: int) -> bool:
    return (
        isinstance(value, list) and len(value) == size and
        all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item)
            for item in value)
    )


def validate_step(
        episode: Path, line: int, row: dict[str, Any], metadata: dict[str, Any],
        issues: list[str]) -> tuple[str, bool, float] | None:
    state = row.get('robot_state')
    if not finite_vector(state, STATE_SIZE):
        issue(issues, episode, line, 'robot_state must contain 25 finite values')
        return None
    if not 0.0 <= state[22] <= 1.0 or not 0.0 <= state[23] <= 1.0 or state[24] not in (0.0, 1.0):
        issue(issues, episode, line, 'continuous state coverage, match ratio or last-goal flag is invalid')

    action = row.get('action')
    if not finite_vector(action, ACTION_SIZE):
        issue(issues, episode, line, 'action must contain dx, dy and dyaw')
        return None
    translation = math.hypot(action[0], action[1])
    if translation > MAX_TRANSLATION_M + 1e-6 or abs(action[2]) > MAX_YAW_RAD + 1e-6:
        issue(issues, episode, line, 'action exceeds the continuous local-goal bounds')
    if translation < 0.05 and abs(action[2]) < 0.05:
        issue(issues, episode, line, 'action is a non-progressing local goal')
    if not isinstance(row.get('image'), str) or not (episode / row['image']).is_file():
        issue(issues, episode, line, 'referenced RGB image is missing')
    if not isinstance(row.get('instruction'), str) or not row['instruction'].strip():
        issue(issues, episode, line, 'stage instruction is required')

    oracle = row.get('oracle')
    if not isinstance(oracle, dict):
        issue(issues, episode, line, 'oracle metadata is required')
        return None
    visible = oracle.get('target_visible')
    pixels = oracle.get('target_label_pixels')
    stage = oracle.get('stage')
    if not isinstance(visible, bool) or not isinstance(pixels, int) or pixels < 0:
        issue(issues, episode, line, 'oracle visibility and pixel count are invalid')
        return None
    if visible != (pixels > 0):
        issue(issues, episode, line, 'target_visible must match target_label_pixels')
    if stage not in {'search', 'approach'}:
        issue(issues, episode, line, 'oracle.stage must be search or approach')
        return None
    if visible != (stage == 'approach') or (state[16] == 1.0) != visible:
        issue(issues, episode, line, 'stage and target-valid state must match visibility')
    if not metadata.get('target_present') and visible:
        issue(issues, episode, line, 'target-absent episode must not expose target')
    return stage, visible, translation


def main() -> int:
    args = parse_args()
    raw_root = args.raw_root.resolve()
    episodes_root = raw_root / 'episodes'
    if not episodes_root.is_dir():
        raise SystemExit(f'raw episodes directory is missing: {episodes_root}')
    if args.report_dir.exists():
        raise SystemExit(f'report directory already exists: {args.report_dir}')

    issues: list[str] = []
    split_counts: collections.Counter[str] = collections.Counter()
    semantic_counts: collections.Counter[str] = collections.Counter()
    translations: list[float] = []
    seeds: set[int] = set()
    episodes = sorted(path for path in episodes_root.iterdir() if path.is_dir())
    for episode in episodes:
        try:
            metadata = json.loads((episode / 'episode.json').read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            issues.append(f'{episode.name}: invalid episode.json: {error}')
            continue
        if metadata.get('policy_interface') != 'continuous_local_goal':
            issues.append(f'{episode.name}: policy_interface must be continuous_local_goal')
        split = metadata.get('split')
        if split not in SPLIT_RATIOS:
            issues.append(f'{episode.name}: invalid split')
        else:
            split_counts[split] += 1
        seed = metadata.get('seed')
        if not isinstance(seed, int) or seed in seeds:
            issues.append(f'{episode.name}: seed must be unique')
        else:
            seeds.add(seed)
        target_present = metadata.get('target_present')
        if not isinstance(target_present, bool):
            issues.append(f'{episode.name}: target_present must be boolean')
            target_present = False
        if not isinstance(metadata.get('coverage_complete'), bool):
            issues.append(f'{episode.name}: coverage_complete must be boolean')
        try:
            rows = (episode / 'steps.jsonl').read_text(encoding='utf-8').splitlines()
        except OSError as error:
            issues.append(f'{episode.name}: missing steps.jsonl: {error}')
            continue
        records = [
            validate_step(episode, line, json.loads(row), metadata, issues)
            for line, row in enumerate(rows, 1) if row.strip()
        ]
        records = [record for record in records if record is not None]
        if not records:
            issues.append(f'{episode.name}: no valid continuous Oracle steps')
            continue
        translations.extend(record[2] for record in records)
        has_approach = any(record[0] == 'approach' for record in records)
        if target_present:
            semantic_counts['target_present'] += 1
            if records[-1][0] != 'approach' or not records[-1][1] or not has_approach:
                issues.append(f'{episode.name}: target-present episode must end with visible approach')
            else:
                semantic_counts['target_present_approached'] += 1
        else:
            semantic_counts['target_absent'] += 1
            if has_approach or metadata.get('coverage_complete') is not True:
                issues.append(f'{episode.name}: target-absent episode must cover search without approach')
            else:
                semantic_counts['target_absent_covered'] += 1

    for split, ratio in SPLIT_RATIOS.items():
        if episodes and abs(split_counts[split] - len(episodes) * ratio) > 1.0:
            issues.append(f'split {split} is not within one episode of the {ratio:.0%} target')
    report = {
        'schema_version': REPORT_SCHEMA_VERSION,
        'policy_interface': 'continuous_local_goal',
        'raw_root': str(raw_root),
        'raw_manifest_sha256': raw_manifest_sha256(raw_root),
        'episodes': len(episodes),
        'split_counts': dict(split_counts),
        'semantic_counts': dict(semantic_counts),
        'samples': len(translations),
        'mean_translation_m': sum(translations) / len(translations) if translations else 0.0,
        'valid': not issues,
        'issues': issues,
    }
    args.report_dir.mkdir(parents=True)
    (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    lines = [
        '# 连续局部目标 VLA 原始数据质量报告', '',
        f'- Episode：{report["episodes"]}', f'- 样本：{report["samples"]}',
        f'- 平均局部位移：{report["mean_translation_m"]:.3f} m',
        f'- 结果：**{"通过" if report["valid"] else "失败"}**', '', '## 问题', '']
    lines.extend(f'- {item}' for item in issues) if issues else lines.append('- 无')
    (args.report_dir / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0 if report['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
