#!/usr/bin/env python3
"""Derive bounded, mean-one action weights from a converted train split."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from pathlib import Path


ACTION_COUNT = 10
DEFAULT_POWER = 0.5
DEFAULT_MAX_MULTIPLIER = 4.0


def action_weights_from_counts(
        action_counts: Mapping[str | int, int], *, power: float = DEFAULT_POWER,
        max_multiplier: float = DEFAULT_MAX_MULTIPLIER) -> tuple[float, ...]:
    """Return inverse-square-root class weights with an original-data mean of one."""
    if not 0.0 < power <= 1.0:
        raise ValueError('power must be in (0, 1]')
    if max_multiplier < 1.0:
        raise ValueError('max_multiplier must be at least one')

    counts: list[int] = []
    for action_id in range(ACTION_COUNT):
        value = action_counts.get(str(action_id), action_counts.get(action_id))
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError(f'action_counts[{action_id}] must be a positive integer')
        counts.append(value)

    maximum = max(counts)
    multipliers = [min(max_multiplier, (maximum / count) ** power) for count in counts]
    mean_multiplier = sum(count * multiplier for count, multiplier in zip(counts, multipliers)) / sum(counts)
    return tuple(multiplier / mean_multiplier for multiplier in multipliers)


def load_train_action_weights(dataset_root: Path) -> tuple[float, ...]:
    """Load class counts only from a converted training split, never validation or test."""
    report_path = dataset_root / 'conversion_report.json'
    try:
        report = json.loads(report_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'cannot read conversion report: {error}') from error
    if report.get('split') != 'train':
        raise ValueError('action-balanced weighting is allowed only for a train split')
    action_counts = report.get('action_counts')
    if not isinstance(action_counts, Mapping):
        raise ValueError('conversion report action_counts must be a mapping')
    return action_weights_from_counts(action_counts)
