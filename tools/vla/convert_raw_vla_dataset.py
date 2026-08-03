#!/usr/bin/env python3
"""Convert validated Gazebo VLA episodes to a local LeRobot v3 dataset."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from validate_raw_vla_dataset import REPORT_SCHEMA_VERSION, raw_manifest_sha256


ACTION_COUNT = 10
STATE_SIZE = 23


def require_validation_report(raw_root: Path, report_path: Path) -> dict[str, Any]:
    """Reject conversion unless the exact Oracle labels passed the current gate."""
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SystemExit(f"invalid validation report: {error}") from error
    if report.get("schema_version") != REPORT_SCHEMA_VERSION:
        raise SystemExit("validation report schema version is unsupported")
    if report.get("valid") is not True:
        raise SystemExit("validation report does not approve this raw dataset")
    if Path(report.get("raw_root", "")).resolve() != raw_root.resolve():
        raise SystemExit("validation report raw_root does not match --raw-root")
    if report.get("raw_manifest_sha256") != raw_manifest_sha256(raw_root):
        raise SystemExit("raw dataset changed after validation; generate a new validation report")
    return report


def features(height: int, width: int) -> dict[str, dict[str, Any]]:
    return {
        "observation.images.front": {
            "dtype": "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (STATE_SIZE,),
            "names": [f"state_{index}" for index in range(STATE_SIZE)],
        },
        "action": {
            "dtype": "float32",
            "shape": (ACTION_COUNT,),
            "names": [f"action_{index}" for index in range(ACTION_COUNT)],
        },
    }


def load_steps(episode_dir: Path) -> Iterable[dict[str, Any]]:
    metadata = json.loads((episode_dir / "episode.json").read_text(encoding="utf-8"))
    steps_path = episode_dir / "steps.jsonl"
    for line_number, line in enumerate(steps_path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        step = json.loads(line)
        step["_episode"] = metadata
        step["_line_number"] = line_number
        yield step


def validate_step(step: dict[str, Any], episode_dir: Path) -> tuple[Path, np.ndarray, np.ndarray, str]:
    image_path = episode_dir / step["image"]
    if not image_path.is_file():
        raise ValueError(f"missing image at {image_path}")
    state = np.asarray(step["robot_state"], dtype=np.float32)
    action_id = step["action_id"]
    instruction = step["instruction"]
    if state.shape != (STATE_SIZE,) or not np.isfinite(state).all():
        raise ValueError(f"invalid state at {episode_dir}:{step['_line_number']}")
    if not isinstance(action_id, int) or not 0 <= action_id < ACTION_COUNT:
        raise ValueError(f"invalid action_id at {episode_dir}:{step['_line_number']}")
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError(f"invalid instruction at {episode_dir}:{step['_line_number']}")
    action = np.zeros(ACTION_COUNT, dtype=np.float32)
    action[action_id] = 1.0
    return image_path, state, action, instruction


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", required=True, type=Path)
    parser.add_argument("--validation-report", required=True, type=Path)
    parser.add_argument("--split", required=True, choices=("train", "validation", "test"))
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    validation_report = require_validation_report(args.raw_root, args.validation_report)

    episode_dirs = sorted((args.raw_root / "episodes").glob("*"))
    selected = [
        path for path in episode_dirs
        if path.is_dir() and json.loads((path / "episode.json").read_text(encoding="utf-8")).get("split") == args.split
    ]
    if not selected:
        raise SystemExit(f"no {args.split} episodes found under {args.raw_root / 'episodes'}")
    if args.output_root.exists():
        if not args.overwrite:
            raise SystemExit(f"output already exists: {args.output_root}; pass --overwrite after review")
        shutil.rmtree(args.output_root)

    first_image = next(load_steps(selected[0]))
    first_path, _, _, _ = validate_step(first_image, selected[0])
    with Image.open(first_path) as image:
        width, height = image.size
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=args.output_root,
        fps=1,
        robot_type="turtlebot3_waffle_pi_cam",
        features=features(height, width),
        use_videos=False,
        image_writer_threads=4,
    )
    action_counts: Counter[int] = Counter()
    try:
        for episode_dir in selected:
            for step in load_steps(episode_dir):
                image_path, state, action, instruction = validate_step(step, episode_dir)
                with Image.open(image_path) as image:
                    frame_image = np.asarray(image.convert("RGB"))
                dataset.add_frame({
                    "observation.images.front": frame_image,
                    "observation.state": state,
                    "action": action,
                    "task": instruction,
                })
                action_counts[int(step["action_id"])] += 1
            dataset.save_episode()
    finally:
        dataset.finalize()

    report = {
        "split": args.split,
        "validation_manifest_sha256": validation_report["raw_manifest_sha256"],
        "episodes": len(selected),
        "frames": sum(action_counts.values()),
        "action_counts": {str(index): action_counts[index] for index in range(ACTION_COUNT)},
    }
    (args.output_root / "conversion_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
