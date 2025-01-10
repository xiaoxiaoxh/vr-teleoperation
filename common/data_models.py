from pydantic import BaseModel, Field
import numpy as np
from enum import Enum, auto
from typing import List, final

class HandMes(BaseModel):
    q: List[float]
    pos: List[float]
    quat: List[float]  # (w, qx, qy, qz)
    thumbTip: List[float]
    indexTip: List[float]
    middleTip: List[float]
    ringTip: List[float]
    pinkyTip: List[float]
    squeeze: float
    cmd: int  # control signal (deprecated now)
    # # for left controller (B, A, joystick, trigger, side_trigger)
    # # for right controller (Y, X, joystick, trigger, side_trigger)
    # buttonState: List[float]

class UnityMes(BaseModel):
    timestamp: float
    valid: bool
    leftHand: HandMes
    rightHand: HandMes

class Arrow(BaseModel):
    start: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    end: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])

class BimanualRobotStates(BaseModel):
    leftRobotTCP: List[float]  # (7) (x, y, z, qw, qx, qy, qz)
    rightRobotTCP: List[float]  # (7) (x, y, z, qw, qx, qy, qz)
    leftGripperState: List[float]  # (2) (width, force)
    rightGripperState: List[float]  # (2) (width, force)

class MoveGripperRequest(BaseModel):
    width: float = 0.05
    velocity: float = 10.0
    force_limit: float = 5.0

class TargetTCPRequest(BaseModel):
    target_tcp: List[float]  # (7) (x, y, z, qw, qx, qy, qz)

class SensorMessage(BaseModel):
    # TODO: adaptable for different dimensions, considering abolishing the 2-D version
    timestamp: float
    leftRobotState: np.ndarray = Field(default_factory=lambda: np.zeros((8, ), dtype=np.float32))  # (6+2) TCP (x, y, z, r, p, y) + gripper (width, force)
    rightRobotState: np.ndarray = Field(default_factory=lambda: np.zeros((8, ), dtype=np.float32))  # (6+2) TCP (x, y, z, r, p, y) + gripper (width, force)
    leftRobotAction: np.ndarray = Field(default_factory=lambda: np.zeros((8, ), dtype=np.float32))  # (6+2) TCP delta value (x, y, z, r, p, y) + gripper (width, force)
    rightRobotAction: np.ndarray = Field(default_factory=lambda: np.zeros((8, ), dtype=np.float32))  # (6+2) TCP delta value (x, y, z, r, p, y) + gripper (width, force)
    externalCameraPointCloud: np.ndarray = Field(default_factory=lambda: np.zeros((2000, 6), dtype=np.float16)) # (H, W, 6) (x, y, z, r, g, b)
    externalCameraRGB: np.ndarray = Field(default_factory=lambda: np.zeros((480, 640, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    leftWristCameraPointCloud: np.ndarray = Field(default_factory=lambda: np.zeros((2000, 6), dtype=np.float16))  # (H, W, 6) (x, y, z, r, g, b)
    leftWristCameraRGB: np.ndarray = Field(default_factory=lambda: np.zeros((480, 640, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    rightWristCameraPointCloud: np.ndarray = Field(default_factory=lambda: np.zeros((2000, 6), dtype=np.float16))  # (H, W, 6) (x, y, z, r, g, b)
    rightWristCameraRGB: np.ndarray = Field(default_factory=lambda: np.zeros((480, 640, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)

    class Config:
        arbitrary_types_allowed = True

class SensorMessageList(BaseModel):
    sensorMessages: List[SensorMessage]

class SensorMode(Enum):
    single_arm_one_realsense = auto()
    single_arm_two_realsense = auto()
    dual_arm_two_realsense = auto()
