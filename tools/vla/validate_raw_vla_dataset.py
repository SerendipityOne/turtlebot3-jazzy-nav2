#!/usr/bin/env python3
"""Validate raw Gazebo Oracle episodes before converting them for LeRobot."""

import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
from typing import Any


ACTION_NAMES = (
    'go_to_candidate_0', 'go_to_candidate_1', 'go_to_candidate_2',
    'scan_left', 'scan_right', 'approach_red', 'approach_green',
    'approach_blue', 'approach_yellow', 'report_not_found')
APPROACH_ACTIONS = {'red': 5, 'green': 6, 'blue': 7, 'yellow': 8}
REPORT_SCHEMA_VERSION = 4
STATE_SIZE = 23
TERMINAL_ACTIONS = set(APPROACH_ACTIONS.values()) | {9}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', required=True, type=Path)
    parser.add_argument('--report-dir', required=True, type=Path)
    return parser.parse_args()


def add_issue(issues: list[str], episode: Path, line_number: int, detail: str) -> None:
    issues.append(f'{episode.name}:step {line_number}: {detail}')


def raw_manifest_sha256(raw_root: Path) -> str:
    """Bind a validation report to Oracle labels without re-reading JPEG payloads."""
    digest = hashlib.sha256()
    episodes_root = raw_root / 'episodes'
    if not episodes_root.is_dir():
        return digest.hexdigest()
    for episode in sorted(path for path in episodes_root.iterdir() if path.is_dir()):
        for name in ('episode.json', 'steps.jsonl'):
            path = episode / name
            digest.update(path.relative_to(raw_root).as_posix().encode('utf-8') + b'\0')
            try:
                digest.update(hashlib.sha256(path.read_bytes()).digest())
            except OSError:
                digest.update(b'MISSING\0')
        frames_root = episode / 'frames'
        if not frames_root.is_dir():
            digest.update((episode.name + '/frames/MISSING\0').encode('utf-8'))
            continue
        for image in sorted(path for path in frames_root.rglob('*') if path.is_file()):
            digest.update(image.relative_to(raw_root).as_posix().encode('utf-8') + b'\0')
            digest.update(f'{image.stat().st_size}\0'.encode('ascii'))
    return digest.hexdigest()


def validate_step(
        episode: Path, line_number: int, row: dict[str, Any], metadata: dict[str, Any],
        issues: list[str]) -> tuple[int | None, bool]:
    action_id = row.get('action_id')
    if not isinstance(action_id, int) or not 0 <= action_id < len(ACTION_NAMES):
        add_issue(issues, episode, line_number, 'action_id must be in [0, 9]')
        return None, False
    state = row.get('robot_state')
    if (not isinstance(state, list) or len(state) != STATE_SIZE or
            any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in state)):
        add_issue(issues, episode, line_number, 'robot_state must contain 23 finite values')
    else:
        scan_progress = state[22]
        if action_id <= 2 and (scan_progress != 1.0 or state[action_id * 4 + 3] != 1.0):
            add_issue(issues, episode, line_number, 'candidate action requires a valid slot after scan completion')
        if action_id == 3 and scan_progress != 0.0:
            add_issue(issues, episode, line_number, 'scan_left requires scan_progress == 0')
        if action_id == 4 and scan_progress != 0.5:
            add_issue(issues, episode, line_number, 'scan_right requires scan_progress == 0.5')
        if action_id == 9 and scan_progress != 1.0:
            add_issue(issues, episode, line_number, 'report_not_found requires scan completion')
    image = row.get('image')
    if not isinstance(image, str) or not (episode / image).is_file():
        add_issue(issues, episode, line_number, 'referenced RGB image is missing')
    if not isinstance(row.get('instruction'), str) or not row['instruction'].strip():
        add_issue(issues, episode, line_number, 'instruction is required')
    oracle = row.get('oracle')
    if not isinstance(oracle, dict):
        add_issue(issues, episode, line_number, 'oracle metadata is required')
        return action_id, False
    target_visible = oracle.get('target_visible')
    target_label_pixels = oracle.get('target_label_pixels')
    coverage_complete = oracle.get('coverage_complete')
    if not isinstance(target_visible, bool):
        add_issue(issues, episode, line_number, 'oracle.target_visible must be boolean')
        target_visible = False
    if (not isinstance(target_label_pixels, int) or isinstance(target_label_pixels, bool) or
            target_label_pixels < 0):
        add_issue(issues, episode, line_number, 'oracle.target_label_pixels must be a non-negative integer')
        target_label_pixels = 0
    if not isinstance(coverage_complete, bool):
        add_issue(issues, episode, line_number, 'oracle.coverage_complete must be boolean')
        coverage_complete = False
    if target_visible != (target_label_pixels > 0):
        add_issue(issues, episode, line_number, 'target_visible must match target_label_pixels')
    if not metadata.get('target_present') and target_visible:
        add_issue(issues, episode, line_number, 'target-absent episode must not mark the target visible')
    if action_id in APPROACH_ACTIONS.values():
        expected_action = APPROACH_ACTIONS.get(metadata.get('target_color'))
        if not metadata.get('target_present') or not target_visible or action_id != expected_action:
            add_issue(issues, episode, line_number, 'approach action violates the target-visibility contract')
    if action_id == 9 and not coverage_complete:
        add_issue(issues, episode, line_number, 'report_not_found requires full candidate coverage')
    return action_id, target_visible


def main() -> int:
    args = parse_args()
    episodes_root = args.raw_root / 'episodes'
    if not episodes_root.is_dir():
        raise SystemExit(f'raw episodes directory is missing: {episodes_root}')
    if args.report_dir.exists():
        raise SystemExit(f'report directory already exists: {args.report_dir}')

    issues: list[str] = []
    split_counts: collections.Counter[str] = collections.Counter()
    action_counts: collections.Counter[str] = collections.Counter()
    semantic_counts: collections.Counter[str] = collections.Counter()
    seeds: set[int] = set()
    episode_count = 0
    for episode in sorted(path for path in episodes_root.iterdir() if path.is_dir()):
        episode_count += 1
        try:
            metadata = json.loads((episode / 'episode.json').read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            issues.append(f'{episode.name}: invalid episode.json: {error}')
            continue
        split = metadata.get('split')
        if split not in {'train', 'validation', 'test'}:
            issues.append(f'{episode.name}: split must be train, validation or test')
        else:
            split_counts[split] += 1
        seed = metadata.get('seed')
        if not isinstance(seed, int) or seed in seeds:
            issues.append(f'{episode.name}: seed must be unique')
        else:
            seeds.add(seed)
        target_color = metadata.get('target_color')
        if target_color not in APPROACH_ACTIONS:
            issues.append(f'{episode.name}: target_color must be one of {sorted(APPROACH_ACTIONS)}')
        target_present = metadata.get('target_present')
        if not isinstance(target_present, bool):
            issues.append(f'{episode.name}: target_present must be boolean')
            target_present = False
        try:
            rows = (episode / 'steps.jsonl').read_text(encoding='utf-8').splitlines()
        except OSError as error:
            issues.append(f'{episode.name}: missing steps.jsonl: {error}')
            continue
        if not rows:
            issues.append(f'{episode.name}: no Oracle steps')
            continue
        step_records: list[tuple[int, bool]] = []
        for line_number, text in enumerate(rows, 1):
            try:
                action_id, target_visible = validate_step(
                    episode, line_number, json.loads(text), metadata, issues)
            except json.JSONDecodeError as error:
                add_issue(issues, episode, line_number, f'invalid JSON: {error}')
                continue
            if action_id is not None:
                action_counts[ACTION_NAMES[action_id]] += 1
                step_records.append((action_id, target_visible))
        if not step_records:
            issues.append(f'{episode.name}: no valid Oracle steps')
            continue
        terminal_action, terminal_visible = step_records[-1]
        for record_index, (action_id, _) in enumerate(step_records[:-1], 1):
            if action_id in TERMINAL_ACTIONS:
                add_issue(issues, episode, record_index, 'terminal action must be the final step')
        approach_count = sum(action_id in APPROACH_ACTIONS.values() for action_id, _ in step_records)
        report_count = sum(action_id == 9 for action_id, _ in step_records)
        any_visible = any(target_visible for _, target_visible in step_records)
        expected_action = APPROACH_ACTIONS.get(target_color)
        if target_present:
            semantic_counts['target_present'] += 1
            if terminal_action == expected_action and terminal_visible and approach_count == 1 and report_count == 0:
                semantic_counts['target_present_approached'] += 1
            else:
                semantic_counts['target_present_missed'] += 1
                issues.append(f'{episode.name}: target-present episode must terminate with one visible matching approach action')
        else:
            semantic_counts['target_absent'] += 1
            if terminal_action == 9 and approach_count == 0 and not any_visible:
                semantic_counts['target_absent_reported'] += 1
            else:
                semantic_counts['target_absent_violation'] += 1
                issues.append(f'{episode.name}: target-absent episode must terminate with report_not_found only')

    expected = {'train': 0.70, 'validation': 0.15, 'test': 0.15}
    if episode_count:
        for split, ratio in expected.items():
            if abs(split_counts[split] - episode_count * ratio) > 1.0:
                issues.append(f'split {split} is not within one episode of the {ratio:.0%} target')
    report = {
        'schema_version': REPORT_SCHEMA_VERSION,
        'raw_root': str(args.raw_root.resolve()),
        'raw_manifest_sha256': raw_manifest_sha256(args.raw_root),
        'episodes': episode_count,
        'split_counts': dict(split_counts),
        'action_counts': {name: action_counts[name] for name in ACTION_NAMES},
        'semantic_counts': dict(semantic_counts),
        'valid': not issues,
        'issues': issues,
    }
    args.report_dir.mkdir(parents=True)
    (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    markdown = [
        '# VLA 原始数据质量报告', '', f'- 原始目录：`{report["raw_root"]}`',
        f'- Episode：{episode_count}', f'- 结果：**{"通过" if not issues else "失败"}**', '',
        '- 接近标签规则：目标语义可见即可接近；运行时仍需通过 YOLO 与 Nav2 安全契约。', '',
        '## 划分', '', '| split | episodes |', '| --- | ---: |',
        *(f'| {split} | {split_counts[split]} |' for split in ('train', 'validation', 'test')), '',
        '## 动作分布', '', '| action | samples |', '| --- | ---: |',
        *(f'| {name} | {action_counts[name]} |' for name in ACTION_NAMES), '',
        '## 语义终止', '', '| metric | episodes |', '| --- | ---: |',
        *(f'| {name} | {semantic_counts[name]} |' for name in (
            'target_present', 'target_present_approached', 'target_present_missed',
            'target_absent', 'target_absent_reported', 'target_absent_violation')), '',
        '## 问题', '']
    markdown.extend(f'- {issue}' for issue in issues) if issues else markdown.append('- 无')
    (args.report_dir / 'report.md').write_text('\n'.join(markdown) + '\n', encoding='utf-8')
    return 0 if not issues else 1


if __name__ == '__main__':
    raise SystemExit(main())
