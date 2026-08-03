#!/usr/bin/env bash
set -euo pipefail

: "${VLA_CONTINUOUS_TRAIN_ROOT:?Set VLA_CONTINUOUS_TRAIN_ROOT to the converted train dataset root.}"
: "${VLA_CONTINUOUS_OUTPUT_DIR:?Set VLA_CONTINUOUS_OUTPUT_DIR to an ignored output directory.}"
: "${VLA_CONTINUOUS_TRAIN_REPO_ID:=local/tb3_ball_continuous_vla_train}"

python tools/vla/train_smolvla_continuous.py \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id="$VLA_CONTINUOUS_TRAIN_REPO_ID" \
  --dataset.root="$VLA_CONTINUOUS_TRAIN_ROOT" \
  --policy.chunk_size=1 \
  --policy.n_action_steps=1 \
  --policy.input_features=null \
  --policy.freeze_vision_encoder=true \
  --policy.train_expert_only=true \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --save_checkpoint_to_hub=false \
  --batch_size=8 \
  --steps=10000 \
  --output_dir="$VLA_CONTINUOUS_OUTPUT_DIR"
