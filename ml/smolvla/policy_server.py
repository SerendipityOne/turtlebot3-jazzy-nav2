"""Loopback-only SmolVLA inference adapter for high-level navigation skills."""

import base64
import io
import logging
import os
from contextlib import asynccontextmanager

import numpy as np
import torch
from fastapi import FastAPI, HTTPException
from lerobot.policies import make_pre_post_processors
from lerobot.policies.smolvla import SmolVLAPolicy
from PIL import Image
from pydantic import BaseModel, Field


SKILLS = (
    'go_to_viewpoint',
    'rotate_scan',
    'approach_target',
    'report_not_found',
    'stop',
)


class PolicyRequest(BaseModel):
    instruction: str = Field(min_length=1, max_length=512)
    image_jpeg_base64: str
    robot_state: list[float] = Field(min_length=3, max_length=32)


class PolicyResponse(BaseModel):
    skill: str
    confidence: float


class PolicyRuntime:
    def __init__(self):
        self.device = torch.device(os.environ.get('SMOLVLA_DEVICE', 'cuda'))
        self.policy_path = os.environ['SMOLVLA_POLICY_PATH']
        self.policy = SmolVLAPolicy.from_pretrained(self.policy_path).to(self.device)
        self.policy.eval()
        self.preprocess, self.postprocess = make_pre_post_processors(
            self.policy.config,
            self.policy_path,
            preprocessor_overrides={
                'device_processor': {'device': str(self.device)},
            },
        )

    @torch.inference_mode()
    def select(self, request: PolicyRequest) -> PolicyResponse:
        try:
            image_bytes = base64.b64decode(request.image_jpeg_base64, validate=True)
            rgb = np.asarray(Image.open(io.BytesIO(image_bytes)).convert('RGB'))
        except Exception as error:
            raise ValueError('image_jpeg_base64 is not a valid JPEG image') from error

        image = torch.from_numpy(rgb.copy()).permute(2, 0, 1).float() / 255.0
        observation = {
            'observation.images.front': image.unsqueeze(0),
            'observation.state': torch.tensor(
                [request.robot_state], dtype=torch.float32),
            'task': [request.instruction],
        }
        action = self.postprocess(self.policy.select_action(self.preprocess(observation)))
        scores = action.detach().float().cpu().reshape(-1)[:len(SKILLS)]
        probabilities = torch.softmax(scores, dim=0)
        index = int(torch.argmax(probabilities))
        return PolicyResponse(skill=SKILLS[index], confidence=float(probabilities[index]))


runtime: PolicyRuntime | None = None


@asynccontextmanager
async def lifespan(_: FastAPI):
    global runtime
    runtime = PolicyRuntime()
    yield
    runtime = None


app = FastAPI(title='TurtleBot3 SmolVLA Skill Policy', lifespan=lifespan)


@app.get('/health')
def health():
    return {'ready': runtime is not None, 'skills': SKILLS}


@app.post('/v1/select_skill', response_model=PolicyResponse)
def select_skill(request: PolicyRequest):
    if runtime is None:
        raise HTTPException(status_code=503, detail='policy is not loaded')
    try:
        return runtime.select(request)
    except ValueError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    except Exception as error:
        logging.exception('SmolVLA policy inference failed')
        raise HTTPException(status_code=500, detail='policy inference failed') from error
