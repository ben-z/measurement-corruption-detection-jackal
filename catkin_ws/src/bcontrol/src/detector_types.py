import numpy as np

from enum import Enum
from typing import List, TypedDict, Union, Optional

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

MODEL_STATE = Union[DifferentialDriveStates,KinematicBicycleStates]

class SensorType(str, Enum):
    ODOMETRY = "ODOMETRY"
    IMU = "IMU"

class SensorConfig(TypedDict):
    name: str
    topic: str
    type: SensorType
    measured_states: List[MODEL_STATE]
    transform_to_solve_frame: Optional[bool]

MODEL_INPUT = Union[DifferentialDriveInputs,KinematicBicycleInputs]

class InputType(str, Enum):
    ACCEL_STAMPED = "ACCEL_STAMPED"
    ACCEL = "ACCEL"

class InputConfig(TypedDict):
    name: str
    topic: str
    type: InputType
    measured_inputs: List[MODEL_INPUT]

class ModelConfig(TypedDict):
    model_type: ModelType
    N: int
    dt: float
    max_update_delay: float
    sensors: List[SensorConfig]
    inputs: List[InputConfig]
    solve_frame: str

class DetectorData(TypedDict):
    C: List[np.ndarray]
    Y: List[np.ndarray]
    U: List[np.ndarray]
    X: List[np.ndarray]
    # sensors and inputs are lists of lists, where the outer list is the time
    sensors_present: List[List[SensorConfig]]
    inputs_present: List[List[InputConfig]]
