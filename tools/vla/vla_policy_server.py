#!/usr/bin/env python3
"""Loopback-only SmolVLA service for TurtleBot3 policy inference."""

from __future__ import annotations

import argparse
import base64
import binascii
import io
import json
import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

# The policy server is an operational component: it must never fetch weights
# while a robot is running.  Set these before importing LeRobot/Transformers.
os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"

import numpy as np
from PIL import Image
import torch
from lerobot.policies import make_pre_post_processors
from lerobot.policies.smolvla import SmolVLAPolicy
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.utils import build_inference_frame


DISCRETE_ACTION_COUNT = 10
DISCRETE_STATE_SIZE = 23
CONTINUOUS_ACTION_SIZE = 3
CONTINUOUS_STATE_SIZE = 25
MAX_LOCAL_TRANSLATION_M = 0.75
MAX_LOCAL_YAW_RAD = 0.7853981633974483
MAX_REQUEST_BYTES = 2 * 1024 * 1024


def interface_spec(interface: str) -> tuple[int, int, int, str]:
    """Return the immutable wire contract for one supported policy interface."""
    if interface == "discrete_skill":
        return DISCRETE_STATE_SIZE, DISCRETE_ACTION_COUNT, 1, "/select_action"
    if interface == "continuous_local_goal":
        return CONTINUOUS_STATE_SIZE, CONTINUOUS_ACTION_SIZE, 2, "/predict_local_goal"
    raise ValueError("interface must be discrete_skill or continuous_local_goal")


def dataset_features(
    image_height: int, image_width: int, state_size: int, action_size: int
) -> dict[str, dict[str, Any]]:
    """Return the exact feature contract used by dataset conversion and inference."""
    return {
        "observation.images.front": {
            "dtype": "image",
            "shape": (image_height, image_width, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (state_size,),
            "names": [f"state_{index}" for index in range(state_size)],
        },
        "action": {
            "dtype": "float32",
            "shape": (action_size,),
            "names": [f"action_{index}" for index in range(action_size)],
        },
    }


def decode_request(
    payload: bytes, schema_version: int, state_size: int
) -> tuple[str, np.ndarray, np.ndarray]:
    """Validate the C++ wire contract before any GPU work is scheduled."""
    if len(payload) > MAX_REQUEST_BYTES:
        raise ValueError("request exceeds the 2 MiB loopback limit")
    request = json.loads(payload)
    if request.get("schema_version") != schema_version:
        raise ValueError("unsupported schema_version")
    instruction = request.get("instruction")
    state = request.get("robot_state")
    encoded_image = request.get("image_jpeg_base64")
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError("instruction must be a non-empty string")
    if not isinstance(state, list) or len(state) != state_size:
        raise ValueError(f"robot_state must contain {state_size} values")
    state_array = np.asarray(state, dtype=np.float32)
    if not np.isfinite(state_array).all():
        raise ValueError("robot_state contains non-finite values")
    if not isinstance(encoded_image, str):
        raise ValueError("image_jpeg_base64 is required")
    image_bytes = base64.b64decode(encoded_image, validate=True)
    if not image_bytes or len(image_bytes) > MAX_REQUEST_BYTES:
        raise ValueError("JPEG payload is invalid")
    with Image.open(io.BytesIO(image_bytes)) as image:
        rgb = np.asarray(image.convert("RGB"))
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("JPEG must decode to RGB")
    return instruction, rgb, state_array


def decode_action(action: np.ndarray) -> tuple[int, float, float]:
    """Map a continuous SmolVLA action endpoint to the nearest discrete prototype."""
    vector = np.asarray(action, dtype=np.float32).reshape(-1)
    if vector.size != DISCRETE_ACTION_COUNT or not np.isfinite(vector).all():
        raise ValueError("policy returned an invalid action vector")
    prototypes = np.eye(DISCRETE_ACTION_COUNT, dtype=np.float32)
    distances = np.linalg.norm(prototypes - vector, axis=1)
    ordered = np.argsort(distances)
    action_id = int(ordered[0])
    return action_id, float(distances[ordered[0]]), float(distances[ordered[1]] - distances[ordered[0]])


def decode_local_goal(action: np.ndarray) -> tuple[float, float, float]:
    """Reject unsafe continuous policy output before it reaches the ROS client."""
    vector = np.asarray(action, dtype=np.float32).reshape(-1)
    if vector.size != CONTINUOUS_ACTION_SIZE or not np.isfinite(vector).all():
        raise ValueError("policy returned an invalid continuous action vector")
    delta_x_m, delta_y_m, delta_yaw_rad = (float(value) for value in vector)
    if np.hypot(delta_x_m, delta_y_m) > MAX_LOCAL_TRANSLATION_M:
        raise ValueError("policy local translation exceeds 0.75 m")
    if abs(delta_yaw_rad) > MAX_LOCAL_YAW_RAD:
        raise ValueError("policy local yaw exceeds pi/4")
    return delta_x_m, delta_y_m, delta_yaw_rad


def validate_vlm_model_dir(vlm_model_dir: Path) -> Path:
    """Return a complete local SmolVLM snapshot path or fail before GPU setup."""
    resolved_dir = vlm_model_dir.expanduser().resolve()
    missing_files = [
        filename
        for filename in ("config.json", "model.safetensors")
        if not (resolved_dir / filename).is_file()
    ]
    if missing_files:
        missing = ", ".join(missing_files)
        raise ValueError(f"local SmolVLM directory is incomplete: {resolved_dir} (missing {missing})")
    return resolved_dir


class PolicyRuntime:
    def __init__(
        self,
        checkpoint: str,
        vlm_model_dir: Path,
        device: str,
        seed: int,
        model_version: str,
        interface: str,
    ) -> None:
        self.state_size, self.action_size, self.schema_version, self.endpoint_path = interface_spec(interface)
        self.interface = interface
        local_vlm_dir = validate_vlm_model_dir(vlm_model_dir)
        model_config = SmolVLAConfig.from_pretrained(checkpoint, local_files_only=True)
        model_config.vlm_model_name = str(local_vlm_dir)
        print(f"Loading VLA checkpoint {checkpoint} with local SmolVLM {local_vlm_dir}", flush=True)
        self.device = torch.device(device)
        self.model = SmolVLAPolicy.from_pretrained(
            checkpoint, config=model_config, local_files_only=True
        )
        if self.model.config.chunk_size != 1 or self.model.config.n_action_steps != 1:
            raise ValueError("checkpoint must use chunk_size=1 and n_action_steps=1")
        if self.model.config.action_feature.shape[0] != self.action_size:
            raise ValueError(f"checkpoint action dimension must be {self.action_size}")
        self.preprocess, self.postprocess = make_pre_post_processors(
            self.model.config,
            checkpoint,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        generator = torch.Generator(device=self.device)
        generator.manual_seed(seed)
        self.fixed_noise = torch.normal(
            mean=0.0,
            std=1.0,
            size=(1, 1, self.model.config.max_action_dim),
            generator=generator,
            dtype=torch.float32,
            device=self.device,
        )
        self.model_version = model_version
        self.lock = threading.Lock()

    def select(self, instruction: str, image: np.ndarray, state: np.ndarray) -> dict[str, Any]:
        started = time.perf_counter()
        raw_observation: dict[str, Any] = {"front": image}
        raw_observation.update({f"state_{index}": value for index, value in enumerate(state)})
        features = dataset_features(image.shape[0], image.shape[1], self.state_size, self.action_size)
        with self.lock, torch.inference_mode():
            frame = build_inference_frame(
                observation=raw_observation,
                ds_features=features,
                device=self.device,
                task=instruction,
                robot_type="turtlebot3_waffle_pi_cam",
            )
            frame = self.preprocess(frame)
            self.model.reset()
            action = self.postprocess(self.model.select_action(frame, noise=self.fixed_noise))
        inference_milliseconds = round((time.perf_counter() - started) * 1000.0, 3)
        if self.interface == "discrete_skill":
            action_id, distance, gap = decode_action(action.detach().cpu().numpy())
            return {
                "schema_version": 1,
                "action_id": action_id,
                "prototype_distance": distance,
                "prototype_gap": gap,
                "model_version": self.model_version,
                "inference_milliseconds": inference_milliseconds,
            }
        delta_x_m, delta_y_m, delta_yaw_rad = decode_local_goal(action.detach().cpu().numpy())
        return {
            "schema_version": 2,
            "delta_x_m": delta_x_m,
            "delta_y_m": delta_y_m,
            "delta_yaw_rad": delta_yaw_rad,
            "model_version": self.model_version,
            "inference_milliseconds": inference_milliseconds,
        }


class PolicyHandler(BaseHTTPRequestHandler):
    runtime: PolicyRuntime

    def do_POST(self) -> None:  # noqa: N802
        if self.path != self.runtime.endpoint_path:
            self.send_error(HTTPStatus.NOT_FOUND, f"only {self.runtime.endpoint_path} is available")
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0 or content_length > MAX_REQUEST_BYTES:
                raise ValueError("invalid Content-Length")
            instruction, image, state = decode_request(
                self.rfile.read(content_length), self.runtime.schema_version, self.runtime.state_size
            )
            response = self.runtime.select(instruction, image, state)
            self._send_json(HTTPStatus.OK, response)
        except (ValueError, json.JSONDecodeError, binascii.Error) as error:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
        except Exception as error:  # The C++ client treats non-2xx responses as fail-closed.
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(error)})

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: HTTPStatus, body: dict[str, Any]) -> None:
        encoded = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--vlm-model-dir", required=True, type=Path)
    parser.add_argument("--port", type=int, default=8089)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--seed", type=int, default=20260727)
    parser.add_argument("--model-version", default="tb3-ball-vla-v1")
    parser.add_argument(
        "--interface", choices=("discrete_skill", "continuous_local_goal"), default="discrete_skill"
    )
    args = parser.parse_args()
    if not args.checkpoint.exists() or not 1 <= args.port <= 65535:
        raise SystemExit("checkpoint must exist and port must be within 1..65535")
    try:
        local_vlm_dir = validate_vlm_model_dir(args.vlm_model_dir)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    PolicyHandler.runtime = PolicyRuntime(
        str(args.checkpoint), local_vlm_dir, args.device, args.seed, args.model_version, args.interface
    )
    server = ThreadingHTTPServer(("127.0.0.1", args.port), PolicyHandler)
    print(
        f"VLA policy server listening on http://127.0.0.1:{args.port}"
        f"{PolicyHandler.runtime.endpoint_path}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
