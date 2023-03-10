#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, AccelStamped
from sensor_msgs.msg import Imu
from time import sleep
from utils import Path, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, typeguard
from typing import Optional, TypedDict, List, Any, Callable
from threading import Lock
from planner import PLANNER_PATH_CLOSED
from enum import Enum
from detector_utils import ModelConfig, SensorConfig, InputConfig, ModelType, MODEL_STATE, DifferentialDriveStates, KinematicBicycleStates, DetectorData, imu_msg_to_state, odometry_msg_to_state, get_model_states, get_model_inputs, accel_stamped_msg_to_input

NODE_NAME = 'bdetect'
DETECTOR_SOLVE_HZ = 1.0 # Hz
DETECTOR_SOLVE_PERIOD = 1.0 / DETECTOR_SOLVE_HZ # seconds

# TODO: Implement the detector
# Inputs: odom from IMU, x, y, orientation from GPS, path from the planner
# Outputs:
# - Whether each sensor is corrupted
# - Passthrough of sensors that are not corrupted

class SensorState(TypedDict):
    config: SensorConfig
    C_block: np.ndarray
    data: Optional[Any]
    extract_measurements_fn: Callable[[Any], np.ndarray]

class InputState(TypedDict):
    config: InputConfig
    data: Optional[Any]
    extract_input_fn: Callable[[Any,np.ndarray], None]

class State(TypedDict):
    estimate: Optional[Odometry]
    path: Optional[Path]
    lock: Lock
    sensors: List[SensorState]
    inputs: List[InputState]
    detector_data: Optional[DetectorData]
    sensor_validity_pub: Optional[rospy.Publisher]

state: State = {
    'estimate': None,
    'path': None,
    'lock': Lock(),
    'sensors': [],
    'inputs': [],
    'detector_data': None,
    'sensor_validity_pub': None,
}

def odom_callback(estimate: Odometry):
    with state['lock']:
        state['estimate'] = estimate

def planner_path_callback(path_msg: PoseArray):
    with state['lock']:
        state['path'] = Path.from_pose_array(path_msg, closed=PLANNER_PATH_CLOSED)

def tick_detector():
    with state['lock']:
        estimate = state['estimate']
        path = state['path']
    
    if estimate is None or path is None:
        rospy.logwarn(f"Detector: waiting for inputs (availability: estimate={not not estimate}, path={not not path})")
        return
    
    rospy.loginfo(f"Detector: got inputs (estimate={estimate}, path={path})")


def construct_C_block(model_type: ModelType, measured_states: List[MODEL_STATE]):
    """
    Constructs a block of C for a given model type and list of measured states.
    """
    state_list = get_model_states(model_type)

    C_block = np.zeros((len(measured_states), len(state_list)))
    for i, state in enumerate(measured_states):
        state_idx = state_list.index(state) # type: ignore
        C_block[i, state_idx] = 1
    return C_block
    
def preprocess_sensor(model_type: ModelType, sensor_config: SensorConfig):
    """
    Takes in a sensor config and populates the state with appropriate data structures.
    Also returns a callback function that should be called when a message is received from the sensor.
    """
    if sensor_config['type'] == 'ODOMETRY':
        data_class = Odometry
        msg_to_state_fn = odometry_msg_to_state
    elif sensor_config['type'] == 'IMU':
        data_class = Imu
        msg_to_state_fn = imu_msg_to_state
    else:
        raise Exception(f"Unknown sensor type {sensor_config['type']}")

    C_block = construct_C_block(model_type, sensor_config['measured_states'])

    def extract_measurements(sensor_msg):
        """
        Extracts the measurements from a sensor message.
        """
        partial_state = msg_to_state_fn(sensor_msg, model_type)

        return C_block @ partial_state

    ss = SensorState(config=sensor_config, C_block=C_block, data=None, extract_measurements_fn=extract_measurements)
    state['sensors'].append(ss)

    def sensor_callback(sensor_msg):
        ss['data'] = sensor_msg

    return data_class, sensor_callback

def preprocess_input(model_type: ModelType, input_config: InputConfig):
    """
    Takes in an input config and populates the state with appropriate data structures.
    Also returns a callback function that should be called when a message is received from the input.
    """
    if input_config['type'] == 'ACCEL_STAMPED':
        data_class = AccelStamped
        extract_input_fn = accel_stamped_msg_to_input
    else:
        raise Exception(f"Unknown input type {input_config['type']}")

    model_inputs = get_model_inputs(model_type)

    def extract_input(input_msg, input_arr: np.ndarray):
        """
        Extracts the input from an input message and puts it into the input array.
        """
        extracted_input = extract_input_fn(input_msg, model_type)

        for input in input_config['measured_inputs']:
            input_idx = model_inputs.index(input)
            input_arr[input_idx] = extracted_input[input_idx]

    ss = InputState(config=input_config, data=None, extract_input_fn=extract_input)
    state['inputs'].append(ss)

    def input_callback(input_msg):
        ss['data'] = input_msg

    return data_class, input_callback

def create_subscriptions(config: ModelConfig):
    """
    Creates subscriptions for each sensor in the config.
    """
    for sensor in config['sensors']:
        data_class, sensor_callback = preprocess_sensor(config['model_type'], sensor)

        rospy.Subscriber(sensor['topic'], data_class, sensor_callback)
    
    for input in config['inputs']:
        data_class, input_callback = preprocess_input(config['model_type'], input)

        rospy.Subscriber(input['topic'], data_class, input_callback)


# def make_empty_C_Y_U(config: ModelConfig):
#     """
#     Creates empty C, Y, and U matrices for the given model config.
#     """
#     # number of time steps in the horizon
#     N = config['N']
#     # number of states
#     n = len(get_model_states(config['model_type']))
#     # number of control inputs
#     p = len(get_model_inputs(config['model_type']))
#     # number of sensor measurements
#     q = sum([len(sensor['measured_states']) for sensor in config['sensors']])
    
#     C = np.zeros((N * q, n))
#     Y = np.zeros((N * q, 1))
#     U = np.zeros((N * p, 1))

#     return C, Y, U


# There should be a few loops here:
# - [x] One loop for each sensor to subscribe to topics and write the latest message to the state
# - [ ] One loop for each control input to the plant
# - [ ] One loop running at the detector's update frequency to gather data and construct the C, Y, and U matrices
        # TODO: Append C_block to C
        # TODO: Append data from sensor_msg to Y
        # TODO: Append metadata (which sensor) to metadata
# - [ ] One loop running at the detector's solve frequency to solve the optimization problem and publish the results

def has_valid_data(sensor: SensorState):
    return sensor['data'] is not None

def loop(event):
    """
    The detector loop has 2 stages:
    1. Update: Gather data from the sensors and construct the C, Y, and U matrices
    2. Solve: Solve the optimization problem
    """
    assert state['detector_data'] is not None, "Detector data should be initialized before the loop starts"

    N = state['detector_data']['config']['N']
    C_blocks = []
    Y_blocks = []
    u = np.zeros((len(get_model_inputs(state['detector_data']['config']['model_type'])), 1))

    sensors_present = []
    inputs_present = []

    for sensor in state['sensors']:
        if sensor['data'] is None:
            continue

        C_block = sensor['C_block']
        sensor_measurements = sensor['extract_measurements_fn'](sensor['data'])

        C_blocks.append(C_block)
        Y_blocks.append(sensor_measurements)

        sensors_present.append(sensor['config'])
    
    for input in state['inputs']:
        if input['data'] is None:
            rospy.logerror(f"Input {input['config']['topic']} has no data. Skipping this iteration.")
            continue

        input['extract_input_fn'](input['data'], u)

        inputs_present.append(input['config'])

    state['detector_data']['C'].append(np.vstack(C_blocks))
    state['detector_data']['Y'].append(np.hstack(Y_blocks).T)
    state['detector_data']['U'].append(u)
    state['detector_data']['sensors_present'].append(sensors_present)
    state['detector_data']['inputs_present'].append(inputs_present)

    # trim the data to the horizon length
    state['detector_data']['C'] = state['detector_data']['C'][-N:]
    state['detector_data']['Y'] = state['detector_data']['Y'][-N:]
    state['detector_data']['U'] = state['detector_data']['U'][-N:]
    state['detector_data']['sensors_present'] = state['detector_data']['sensors_present'][-N:]
    state['detector_data']['inputs_present'] = state['detector_data']['inputs_present'][-N:]

    rospy.loginfo("Finished update stage")

def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Use the current estimate of the robot's state and the planned path for linearization
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PoseArray, planner_path_callback)

    state['sensor_validity_pub'] = rospy.Publisher('/bdetect/sensor_validity', UInt8MultiArray, queue_size=1)

    # Load the configuration and check its type
    config: ModelConfig = typeguard.check_type(rospy.get_param('~bdetect'), ModelConfig)
    rospy.loginfo(f"{NODE_NAME}: loaded configuration {config}")

    state['detector_data'] = DetectorData(config=config, C=[], Y=[], U=[], sensors_present=[], inputs_present=[])

    # Create subscriptions for each sensor
    create_subscriptions(config)

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    rospy.Timer(rospy.Duration(config['dt']), loop)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
