#!/usr/bin/env python3
"""Run the unmodified LeRobot trainer for bounded continuous local-goal actions."""

from __future__ import annotations

import importlib.metadata


def main() -> None:
    if importlib.metadata.version('lerobot') != '0.6.1':
        raise SystemExit('continuous SmolVLA training requires lerobot==0.6.1')
    from lerobot.scripts import lerobot_train
    lerobot_train.main()


if __name__ == '__main__':
    main()
