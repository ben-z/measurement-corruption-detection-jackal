import numpy as np

from enum import Enum
from typing import List, TypedDict, Union, Optional
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, AccelStamped
from tf.transformations import euler_from_quaternion
from utils import flatten

# Value for states/inputs that are not measured
UNMEASURED = 0

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

def get_model_angular_states(model_type: ModelType) -> List[Union[DifferentialDriveStates, KinematicBicycleStates]]:
    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return [DifferentialDriveStates.ORIENTATION, DifferentialDriveStates.ANGULAR_VELOCITY]
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        return [KinematicBicycleStates.ORIENTATION, KinematicBicycleStates.STEERING_ANGLE]
    else:
        raise Exception(f"Unknown model type {model_type}")

def linearize_model(model_type: ModelType, state: np.ndarray, input: np.ndarray, dt: float):
    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return get_linear_differential_drive_model(state, input, dt)
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        raise Exception("Kinematic bicycle model not implemented yet")
        # return get_linear_kinematic_bicycle_model(state, input, dt)
    else:
        raise Exception(f"Unknown model type {model_type}")

def get_linear_differential_drive_model(state: np.ndarray, input: np.ndarray, dt: float):
    """
    Returns the A and B matrices for a discrete linear kinematic differential drive model.
    """
    # Unpack state and input
    # x, y, v, a are the position, velocity, and acceleration of the robot
    # theta is the orientation of the robot
    # omega is the angular velocity of the robot
    # alpha is the angular acceleration of the robot
    x, y, theta, v, omega = state
    a, alpha = input

    A = np.eye(5) + dt * np.array([
        [0, 0, 0, v * np.cos(theta), 0],
        [0, 0, 0, v * np.sin(theta), 0],
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ])

    B = np.array([
        [0, 0],
        [0, 0],
        [0, 0],
        [dt, 0],
        [0, dt],
    ])

    return A, B

def get_model_states(model_type: ModelType) -> List[Union[DifferentialDriveStates, KinematicBicycleStates]]:
    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return list(DifferentialDriveStates)
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        return list(KinematicBicycleStates)
    else:
        raise Exception(f"Unknown model type {model_type}")

def get_model_inputs(model_type: ModelType) -> List[Union[DifferentialDriveInputs, KinematicBicycleInputs]]:
    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return list(DifferentialDriveInputs)
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        return list(KinematicBicycleInputs)
    else:
        raise Exception(f"Unknown model type {model_type}")

def odometry_msg_to_state(odometry_message: Odometry, model_type: ModelType) -> np.ndarray:
    """
    Converts an Odometry message to a state vector.
    UNMEASURED is used for states that are not measured.
    """
    # convert quaternion to euler
    orientation = odometry_message.pose.pose.orientation
    orientation_rpy = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return np.array([
            odometry_message.pose.pose.position.x, # X
            odometry_message.pose.pose.position.y, # Y
            orientation_rpy[2], # ORIENTATION
            odometry_message.twist.twist.linear.x, # VELOCITY
            odometry_message.twist.twist.angular.z, # ANGULAR_VELOCITY
        ])
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        return np.array([
            odometry_message.pose.pose.position.x, # X
            odometry_message.pose.pose.position.y, # Y
            orientation_rpy[2], # ORIENTATION
            odometry_message.twist.twist.linear.x, # VELOCITY
            UNMEASURED, # STEERING_ANGLE
        ])
    else:
        raise Exception(f"Unknown model type {model_type}")

def imu_msg_to_state(imu_message: Imu, model_type: ModelType) -> np.ndarray:
    """
    Converts an IMU message to a state vector.
    UNMEASURED is used for states that are not measured.
    """

    orientation_rpy = euler_from_quaternion([imu_message.orientation.x, imu_message.orientation.y, imu_message.orientation.z, imu_message.orientation.w])

    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return np.array([
            UNMEASURED, # X
            UNMEASURED, # Y
            orientation_rpy[2],  # ORIENTATION
            UNMEASURED, # VELOCITY
            imu_message.angular_velocity.z, # ANGULAR_VELOCITY
        ])
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        return np.array([
            UNMEASURED, # X
            UNMEASURED, # Y
            orientation_rpy[2],  # ORIENTATION
            UNMEASURED, # VELOCITY
            UNMEASURED, # STEERING_ANGLE
        ])
    else:
        raise Exception(f"Unknown model type {model_type}")

def accel_stamped_msg_to_input(accel_stamped_message: AccelStamped, model_type: ModelType) -> np.ndarray:
    """
    Converts an AccelStamped message to an input vector.
    UNMEASURED is used for inputs that are not measured.
    """
    if model_type == ModelType.DIFFERENTIAL_DRIVE:
        return np.array([
            accel_stamped_message.accel.linear.x, # ACCELERATION
            accel_stamped_message.accel.angular.z, # ANGULAR_ACCELERATION
        ])
    elif model_type == ModelType.KINEMATIC_BICYCLE:
        return np.array([
            accel_stamped_message.accel.linear.x, # ACCELERATION
            UNMEASURED, # STEERING_ANGLE_VELOCITY
        ])
    else:
        raise Exception(f"Unknown model type {model_type}")

MODEL_STATE = Union[DifferentialDriveStates,KinematicBicycleStates]

class SensorType(str, Enum):
    ODOMETRY = "ODOMETRY"
    IMU = "IMU"

class SensorConfig(TypedDict):
    topic: str
    type: SensorType
    measured_states: List[MODEL_STATE]
    transform_to_solve_frame: Optional[bool]

MODEL_INPUT = Union[DifferentialDriveInputs,KinematicBicycleInputs]

class InputType(str, Enum):
    ACCEL_STAMPED = "ACCEL_STAMPED"

class InputConfig(TypedDict):
    topic: str
    type: InputType
    measured_inputs: List[MODEL_INPUT]

class ModelConfig(TypedDict):
    model_type: ModelType
    N: int
    dt: float
    sensors: List[SensorConfig]
    inputs: List[InputConfig]
    solve_frame: str

class DetectorData(TypedDict):
    config: ModelConfig
    C: List[np.ndarray]
    Y: List[np.ndarray]
    U: List[np.ndarray]
    X: List[np.ndarray]
    # sensors and inputs are lists of lists, where the outer list is the time
    sensors_present: List[List[SensorConfig]]
    inputs_present: List[List[InputConfig]]

def get_all_measured_states(sensors: List[SensorConfig]):
    return list(flatten([sensor["measured_states"] for sensor in sensors]))

def get_angular_mask(model_type: ModelType, states: List[MODEL_STATE]):
    angular_states = get_model_angular_states(model_type)

    return np.array([state in angular_states for state in states])
