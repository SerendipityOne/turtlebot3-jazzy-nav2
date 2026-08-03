#!/usr/bin/env python3
"""Run LeRobot 0.6.1 training with bounded action-balanced SmolVLA loss weights."""

from __future__ import annotations

import importlib.metadata
import inspect
from pathlib import Path

import torch

from action_balanced_weighting import ACTION_COUNT, load_train_action_weights


REQUIRED_LEROBOT_VERSION = '0.6.1'
WEIGHTING_TYPE = 'tb3_action_balanced'


def require_supported_lerobot() -> None:
    """Keep the wrapper tied to the API whose weighted-loss contract was verified."""
    version = importlib.metadata.version('lerobot')
    if version != REQUIRED_LEROBOT_VERSION:
        raise SystemExit(
            f'{WEIGHTING_TYPE} requires lerobot=={REQUIRED_LEROBOT_VERSION}; found {version}')


def action_ids_from_batch(action: torch.Tensor) -> torch.Tensor:
    """Recover action IDs after LeRobot feature normalization preserves one-hot ordering."""
    if action.ndim != 3 or action.shape[1:] != (1, ACTION_COUNT):
        raise ValueError(
            f'{WEIGHTING_TYPE} requires chunk_size=1 and action shape (batch, 1, {ACTION_COUNT}), '
            f'got {tuple(action.shape)}')
    return action[:, 0, :].argmax(dim=-1)


def install_action_balanced_weighter() -> None:
    """Register the project weighter without modifying LeRobot's installed sources."""
    from lerobot.utils import sample_weighting

    original_factory = sample_weighting.make_sample_weighter

    class ActionBalancedWeighter(sample_weighting.SampleWeighter):
        def __init__(self, action_weights: tuple[float, ...], device: torch.device) -> None:
            self._weights = torch.tensor(action_weights, dtype=torch.float32, device=device)

        def compute_batch_weights(self, batch: dict) -> tuple[torch.Tensor, dict]:
            action = batch.get('action')
            if not isinstance(action, torch.Tensor):
                raise ValueError(f'{WEIGHTING_TYPE} requires a tensor action batch')
            action_ids = action_ids_from_batch(action)
            weights = self._weights.index_select(0, action_ids.to(self._weights.device))
            return weights, {'batch_mean_weight': float(weights.mean().item())}

        def get_stats(self) -> dict:
            return {
                'type': WEIGHTING_TYPE,
                **{f'action_{action_id}_weight': float(weight)
                   for action_id, weight in enumerate(self._weights.tolist())},
            }

    def make_sample_weighter(config, policy, device, dataset_root=None, dataset_repo_id=None):
        if config is None or config.type != WEIGHTING_TYPE:
            return original_factory(config, policy, device, dataset_root, dataset_repo_id)
        if dataset_root is None:
            raise ValueError(f'{WEIGHTING_TYPE} requires --dataset.root')
        if 'reduction' not in inspect.signature(policy.forward).parameters:
            raise ValueError(f'{WEIGHTING_TYPE} requires a policy with forward(..., reduction="none")')
        return ActionBalancedWeighter(load_train_action_weights(Path(dataset_root)), device)

    sample_weighting.make_sample_weighter = make_sample_weighter


def main() -> None:
    require_supported_lerobot()
    install_action_balanced_weighter()
    from lerobot.scripts import lerobot_train
    lerobot_train.main()


if __name__ == '__main__':
    main()
