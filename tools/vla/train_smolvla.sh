#!/usr/bin/env bash
set -euo pipefail

: "${VLA_TRAIN_ROOT:?Set VLA_TRAIN_ROOT to the converted train dataset root.}"
: "${VLA_OUTPUT_DIR:?Set VLA_OUTPUT_DIR to an ignored output directory.}"
: "${VLA_TRAIN_REPO_ID:=local/tb3_ball_vla_train}"

python tools/vla/train_smolvla_balanced.py \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id="$VLA_TRAIN_REPO_ID" \
  --dataset.root="$VLA_TRAIN_ROOT" \
  --policy.chunk_size=1 \
  --policy.n_action_steps=1 \
  --policy.input_features=null \
  --policy.freeze_vision_encoder=true \
  --policy.train_expert_only=true \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --save_checkpoint_to_hub=false \
  --sample_weighting.type=tb3_action_balanced \
  --batch_size=8 \
  --steps=10000 \
  --output_dir="$VLA_OUTPUT_DIR"
