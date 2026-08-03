#!/usr/bin/env python3
"""Small protocol tests that do not load a SmolVLA checkpoint."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np


SERVER_PATH = Path(__file__).with_name("vla_policy_server.py")
SPEC = importlib.util.spec_from_file_location("vla_policy_server", SERVER_PATH)
assert SPEC is not None and SPEC.loader is not None
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)


class ContinuousPolicyProtocolTest(unittest.TestCase):
    def test_continuous_interface_contract(self) -> None:
        self.assertEqual(
            SERVER.interface_spec("continuous_local_goal"),
            (25, 3, 2, "/predict_local_goal"),
        )

    def test_accepts_bounded_continuous_goal(self) -> None:
        goal = SERVER.decode_local_goal(np.array([0.5, 0.25, 0.2], dtype=np.float32))
        self.assertAlmostEqual(goal[0], 0.5)
        self.assertAlmostEqual(goal[1], 0.25)
        self.assertAlmostEqual(goal[2], 0.2)

    def test_rejects_unsafe_continuous_goal(self) -> None:
        with self.assertRaises(ValueError):
            SERVER.decode_local_goal(np.array([0.76, 0.0, 0.0], dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
