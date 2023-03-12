#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, AccelStamped
from sensor_msgs.msg import Imu
from time import sleep
from utils import Path, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, typeguard, add_timer_event_to_diag_status, wrap_angle
from typing import Optional, TypedDict, List, Any, Callable
from threading import Lock
from planner import PLANNER_PATH_CLOSED
from enum import Enum
from detector_utils import ModelConfig, SensorConfig, InputConfig, ModelType, MODEL_STATE, DifferentialDriveStates, KinematicBicycleStates, DetectorData, imu_msg_to_state, odometry_msg_to_state, get_model_states, get_model_inputs, accel_stamped_msg_to_input, linearize_model, get_angular_mask, get_all_measured_states
from copy import deepcopy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from solver_utils import get_evolution_matrices, optimize_l1

NODE_NAME = 'bdetect'
DETECTOR_SOLVE_HZ = 1.0 # Hz
DETECTOR_SOLVE_PERIOD = 1.0 / DETECTOR_SOLVE_HZ # seconds

np.set_printoptions(precision=4,suppress=True)

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
    lock: Lock

class InputState(TypedDict):
    config: InputConfig
    data: Optional[Any]
    extract_input_fn: Callable[[Any,np.ndarray], None]
    lock: Lock

class Diagnostics(TypedDict):
    update_loop: DiagnosticStatus
    solve_loop: DiagnosticStatus

class State(TypedDict):
    estimate: Optional[Odometry]
    path: Optional[Path]
    lock: Lock
    sensors: List[SensorState]
    inputs: List[InputState]
    detector_data: Optional[DetectorData]
    detector_data_lock: Lock
    sensor_validity_pub: Optional[rospy.Publisher]
    diagnostics: Diagnostics
    diagnostics_pub: Optional[rospy.Publisher]
    diagnostics_lock: Lock

state: State = {
    'estimate': None,
    'path': None,
    'lock': Lock(),
    'sensors': [],
    'inputs': [],
    'detector_data': None,
    'detector_data_lock': Lock(),
    'sensor_validity_pub': None,
    'diagnostics': {
        'update_loop': DiagnosticStatus(),
        'solve_loop': DiagnosticStatus(),
    },
    'diagnostics_pub': None,
    'diagnostics_lock': Lock(),
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

    s = SensorState(config=sensor_config, C_block=C_block, data=None, extract_measurements_fn=extract_measurements, lock=Lock())
    state['sensors'].append(s)

    def sensor_callback(sensor_msg):
        with s['lock']:
            s['data'] = sensor_msg

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

    s = InputState(config=input_config, data=None, extract_input_fn=extract_input, lock=Lock())
    state['inputs'].append(s)

    def input_callback(input_msg):
        with s['lock']:
            s['data'] = input_msg

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
# - [x] One loop for each control input to the plant
# - [x] One loop running at the detector's update frequency to gather data and construct the C, Y, and U matrices
        # TODO: Append C_block to C
        # TODO: Append data from sensor_msg to Y
        # TODO: Append metadata (which sensor) to metadata
# - [ ] One loop running at the detector's solve frequency to solve the optimization problem and publish the results

def has_valid_data(sensor: SensorState):
    return sensor['data'] is not None

def update_loop(event: rospy.timer.TimerEvent):
    """
    Gather data from the sensors for the C, Y, and U matrices
    """
    assert state['detector_data'] is not None, "Detector data should be initialized before the loop starts"

    # Populate the diagnostics message
    diag_msg = DiagnosticStatus()
    diag_msg.name = "Detector Update Loop"
    diag_msg.level = DiagnosticStatus.OK
    diag_msg.message = ""
    diag_msg.values = []
    add_timer_event_to_diag_status(diag_msg, event)

    if event.last_duration and event.last_duration > state['detector_data']['config']['dt']:
        diag_msg.level = max(DiagnosticStatus.WARN, diag_msg.level)
        diag_msg.message += "Last update loop took longer than the update period\n"

    N = state['detector_data']['config']['N']
    model_type = state['detector_data']['config']['model_type']
    C_blocks = []
    Y_blocks = []
    u = np.zeros((len(get_model_inputs(model_type)), 1))

    sensors_present = []
    inputs_present = []

    for sensor in state['sensors']:
        with sensor['lock']:
            C_block = sensor['C_block']
            extract_measurements_fn = sensor['extract_measurements_fn']
            config = sensor['config']
            data = sensor['data']
            sensor['data'] = None # clear the data after reading it

        diag_msg.values.append(KeyValue(key=f"{config['topic']} has data", value=str(data is not None)))

        if data is None:
            # If there's no data, then the measurement matrix is all zeros
            C_block = np.zero_like(C_block)
            sensor_measurements = np.zeros((len(config['measured_states']), 1))
        else:
            # TODO check data age
            sensor_measurements = extract_measurements_fn(data)
            sensors_present.append(config)

        C_blocks.append(C_block)
        Y_blocks.append(sensor_measurements)
    
    for input in state['inputs']:
        with input['lock']:
            config = input['config']
            extract_input_fn = input['extract_input_fn']
            data = input['data']
            input['data'] = None # clear the data after reading it

        diag_msg.values.append(KeyValue(key=f"{config['topic']} has data", value=str(data is not None)))

        if data is None:
            rospy.logerror(f"Input {input['config']['topic']} has no data. Skipping this iteration.")
            continue

        # TODO check data age

        extract_input_fn(data, u)

        inputs_present.append(config)

    with state['lock']:
        # TODO check estimate age
        estimate = state['estimate']

    if not sensors_present or not inputs_present or estimate is None:
        rospy.logerror(f"Not enough data is available for an update loop. {len(sensors_present)=}, {len(inputs_present)=}, {estimate=}")

        diag_msg.level = max(DiagnosticStatus.ERROR, diag_msg.level)
        diag_msg.message += "Not enough data is available to update the detector\n"

        with state['diagnostics_lock']:
            state['diagnostics']['update_loop'] = diag_msg
        return

    X_block = odometry_msg_to_state(estimate, model_type)

    with state['detector_data_lock']:
        C = np.vstack(C_blocks)
        Y = np.hstack(Y_blocks).T
        assert C.shape[0] == Y.shape[0], "C and Y should have the same number of rows"

        state['detector_data']['C'].append(C)
        state['detector_data']['Y'].append(Y)
        state['detector_data']['U'].append(u)
        state['detector_data']['X'].append(X_block)
        state['detector_data']['sensors_present'].append(sensors_present)
        state['detector_data']['inputs_present'].append(inputs_present)

        # trim the data to the horizon length
        state['detector_data']['C'] = state['detector_data']['C'][-N:]
        state['detector_data']['Y'] = state['detector_data']['Y'][-N:]
        state['detector_data']['U'] = state['detector_data']['U'][-N:]
        state['detector_data']['X'] = state['detector_data']['X'][-N:]
        state['detector_data']['sensors_present'] = state['detector_data']['sensors_present'][-N:]
        state['detector_data']['inputs_present'] = state['detector_data']['inputs_present'][-N:]

    if diag_msg.message == "":
        diag_msg.message = "ok"

    with state['diagnostics_lock']:
        state['diagnostics']['update_loop'] = diag_msg

    # TODO: Publish the current sensor pass/fail status

def solve_loop(event: rospy.timer.TimerEvent):
    """
    Solve the optimization problem to determine whether each sensor is corrupted.
    """
    assert state['detector_data'] is not None, "Detector data should be initialized before the loop starts"

    # Populate the diagnostics message
    diag_msg = DiagnosticStatus()
    diag_msg.name = "Detector Solve Loop"
    diag_msg.level = DiagnosticStatus.OK
    diag_msg.message = ""
    diag_msg.values = []
    add_timer_event_to_diag_status(diag_msg, event)

    if event.last_duration and event.last_duration > DETECTOR_SOLVE_PERIOD:
        diag_msg.level = max(DiagnosticStatus.WARN, diag_msg.level)
        diag_msg.message += "Last update loop took longer than the update period\n"

    with state['detector_data_lock']:
        detector_data = deepcopy(state['detector_data'])

    model_config = detector_data['config']
    q = sum([len(s['measured_states']) for s in model_config['sensors']])
    p = len(get_model_inputs(model_config['model_type']))
    n = len(get_model_states(model_config['model_type']))
    N = model_config['N']

    if len(detector_data['C']) < N:
        diag_msg.message += "Waiting for sufficient data\n"
        with state['diagnostics_lock']:
            state['diagnostics']['solve_loop'] = diag_msg
        rospy.loginfo("Waiting for sufficient data in the solve loop")
        return

    # Sanity check the data
    for data_name in ['C', 'Y', 'U', 'X']:
        assert len(detector_data[data_name]) == N, f"The number of {data_name} blocks should be equal to the horizon length"

    for data_name in ['C', 'Y']:
        assert all([m.shape[0] == q for m in detector_data[data_name]]), f"The number of rows in each {data_name} should be equal to the number of measurements"

    rospy.logwarn(f"Solving ===============================")

    # Prepare inputs to the optimization problem
    Cs = detector_data['C']
    Ys = detector_data['Y']
    Xs = detector_data['X']

    measured_states = get_all_measured_states(model_config['sensors'])
    angular_outputs_mask = get_angular_mask(model_config['model_type'], measured_states)

    # Linearize the model
    As = []
    Bs = []
    for i in range(N):
        A, B = linearize_model(model_config['model_type'], detector_data['X'][i], detector_data['U'][i], model_config['dt'])

        As.append(A)
        Bs.append(B)
    
    desired_trajectory = np.vstack([C@x for C, x in zip(Cs, Xs)]).T

    print("Xs")
    print(Xs)

    print("desired_trajectory")
    print(desired_trajectory)

    # qxN matrix of measurements
    # TODO: subtract input effects and desired trajectory
    measurements_raw = np.vstack(Ys).T
    measurements = measurements_raw - desired_trajectory
    measurements[angular_outputs_mask, :] = wrap_angle(measurements[angular_outputs_mask, :])

    print("measurements_raw")
    print(measurements_raw)
    print("measurements")
    print(measurements)

    Y = measurements.reshape((q*N,), order='F')
    state_evolution_matrix, Phi = get_evolution_matrices(As[:-1], Cs)

    # Solve the optimization problem
    prob, x0_hat = optimize_l1(n, q, N, Phi, Y)
    # print(prob)
    print(f"{x0_hat.value=} {Xs[0]=}")
    print(f"{Xs[0]-x0_hat.value=}")

    rospy.logwarn(f"Solved ===============================")

    if diag_msg.message == "":
        diag_msg.message = "ok"

    with state['diagnostics_lock']:
        state['diagnostics']['solve_loop'] = diag_msg

def diagnostics_loop(event: rospy.timer.TimerEvent):
    """
    Publish the diagnostics messages
    """
    assert state['diagnostics_pub'] is not None, "Diagnostics publisher should be initialized before the loop starts"

    diagarr_msg = DiagnosticArray()
    diagarr_msg.header.stamp = rospy.Time.now()

    for key, diag in state['diagnostics'].items():
        with state['diagnostics_lock']:
            if diag is not None:
                diagarr_msg.status.append(diag)
                state['diagnostics'][key] = None

    state['diagnostics_pub'].publish(diagarr_msg)

def main():
    # Initialize the node
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Use the current estimate of the robot's state and the planned path for linearization
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PoseArray, planner_path_callback)

    state['sensor_validity_pub'] = rospy.Publisher('/bdetect/sensor_validity', UInt8MultiArray, queue_size=1)
    state['diagnostics_pub'] = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    # Load the configuration and check its type
    config: ModelConfig = typeguard.check_type(rospy.get_param('~bdetect'), ModelConfig)
    rospy.loginfo(f"{NODE_NAME}: loaded configuration {config}")

    state['detector_data'] = DetectorData(config=config, C=[], Y=[], U=[], X=[], sensors_present=[], inputs_present=[])

    # Create subscriptions for each sensor
    create_subscriptions(config)

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # The detector loop has 2 stages:
    # 1. Update: Gather data from the sensors and construct the C, Y, and U matrices
    rospy.Timer(rospy.Duration(config['dt']), update_loop)
    # 2. Solve: Solve the optimization problem
    rospy.Timer(rospy.Duration(DETECTOR_SOLVE_PERIOD), solve_loop)

    # Publish the diagnostics messages
    rospy.Timer(rospy.Duration(min(config['dt'], DETECTOR_SOLVE_PERIOD)), diagnostics_loop)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
