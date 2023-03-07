from typing import Union, List, TypedDict
from enum import Enum

class ModelType(str, Enum):
    DIFFERENTIAL_DRIVE = "DIFFERENTIAL_DRIVE"
    KINEMATIC_BICYCLE = "KINEMATIC_BICYCLE"

class DifferentialDriveStates(str, Enum):
    X = "X"
    Y = "Y"
    ORIENTATION = "ORIENTATION"
    VELOCITY = "VELOCITY"
    ANGULAR_VELOCITY = "ANGULAR_VELOCITY"

class DifferentialDriveInputs(str, Enum):
    ACCELERATION = "ACCELERATION"
    ANGULAR_ACCELERATION = "ANGULAR_ACCELERATION"

class KinematicBicycleStates(str, Enum):
    X = "X"
    Y = "Y"
    ORIENTATION = "ORIENTATION"
    VELOCITY = "VELOCITY"
    STEERING_ANGLE = "STEERING_ANGLE"

class KinematicBicycleInputs(str, Enum):
    ACCELERATION = "ACCELERATION"
    STEERING_ANGLE_VELOCITY = "STEERING_ANGLE_VELOCITY"

STATES = Union[DifferentialDriveStates,KinematicBicycleInputs]

class SensorType(str, Enum):
    ODOMETRY = "ODOMETRY"
    IMU = "IMU"

class SensorConfig(TypedDict):
    topic: str
    type: SensorType
    measured_states: List[STATES]

class Config(TypedDict):
    model_type: ModelType
    sensors: List[SensorConfig]

