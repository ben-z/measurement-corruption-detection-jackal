#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, AccelStamped, Accel
from sensor_msgs.msg import Imu
from time import sleep
from utils import Path, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, typeguard, add_timer_event_to_diag_status, wrap_angle, make_srv_enum_lookup_dict
from typing import Optional, TypedDict, List, Any, Callable
from threading import Lock
from planner import PLANNER_PATH_CLOSED, RADIUS
from enum import Enum
from detector_utils import ModelConfig, SensorConfig, SensorType, InputConfig, ModelType, MODEL_STATE, DifferentialDriveStates, KinematicBicycleStates, DetectorData, imu_msg_to_state, odometry_msg_to_state, get_model_states, get_model_inputs, accel_stamped_msg_to_input, accel_msg_to_input, linearize_model, get_angular_mask, get_all_measured_states, state_to_pose_msg
from copy import deepcopy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from solver_utils import get_evolution_matrices, optimize_l0, optimize_l1, get_l1_objective_fn, get_input_effect_on_state
from transform_frames import TransformFrames
from scipy.linalg import block_diag
from bcontrol.srv import RunSolver, RunSolverRequest, RunSolverResponse
from bcontrol.msg import Path as PathMsg

RUN_SOLVER_RESPONSE_ENUM_TO_STR = make_srv_enum_lookup_dict(RunSolverResponse)

NODE_NAME = 'bdetect'
DETECTOR_SOLVE_HZ = 5 # Hz
DETECTOR_SOLVE_PERIOD = 1.0 / DETECTOR_SOLVE_HZ # seconds
# TODO: this should be encoded in the target path
VELOCITY = 0.5 # m/s

np.set_printoptions(precision=4, suppress=True, linewidth=400)

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
    path_msg: Optional[PathMsg]
    lock: Lock
    sensors: List[SensorState]
    inputs: List[InputState]
    model_config: Optional[ModelConfig]
    detector_data: Optional[DetectorData]
    detector_data_lock: Lock
    sensor_validity_pub: Optional[rospy.Publisher]
    sensor_malfunction_max_magnitude_pub: Optional[rospy.Publisher]
    diagnostics: Diagnostics
    diagnostics_pub: Optional[rospy.Publisher]
    diagnostics_lock: Lock
    transform_frames: Optional[TransformFrames]
    data_pub: Optional[rospy.Publisher]
    solve_data_pub: Optional[rospy.Publisher]
    # used to keep track of the time debt of the update loop, because the difference between the expected and actual time does not go down.
    update_loop_time_debt: float # seconds

state: State = {
    'estimate': None,
    'path_msg': None,
    'lock': Lock(),
    'sensors': [],
    'inputs': [],
    'model_config': None,
    'detector_data': None,
    'detector_data_lock': Lock(),
    'sensor_validity_pub': None,
    'diagnostics': {
        'update_loop': DiagnosticStatus(),
        'solve_loop': DiagnosticStatus(),
    },
    'diagnostics_pub': None,
    'diagnostics_lock': Lock(),
    'transform_frames': None,
    'data_pub': None,
    'solve_data_pub': None,
    'update_loop_time_debt': 0.0,
    'sensor_malfunction_max_magnitude_pub': None,
}

def odom_callback(estimate: Odometry):
    with state['lock']:
        state['estimate'] = estimate

def planner_path_callback(path_msg: PoseArray):
    with state['lock']:
        state['path_msg'] = path_msg

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

def transform_msg_to_solve_frame(solve_frame: str, sensor_type: SensorType, sensor_msg: Any):
    transform_frames = state["transform_frames"]
    if transform_frames is None:
        raise Exception("transform_frames not initialized")

    if sensor_type == SensorType.ODOMETRY:
        return transform_frames.odom_transform(sensor_msg, target_frame=solve_frame)
    else:
        raise Exception(f"Sensor type not yet supported: {sensor_type}")
    
def preprocess_sensor(model_type: ModelType, sensor_config: SensorConfig, solve_frame: str):
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

    if sensor_config.get('transform_to_solve_frame'):
        msg_transform_fn = lambda m: transform_msg_to_solve_frame(solve_frame, sensor_config['type'], m)
    else:
        msg_transform_fn = lambda x: x

    C_block = construct_C_block(model_type, sensor_config['measured_states'])

    def extract_measurements(sensor_msg):
        """
        Extracts the measurements from a sensor message.
        """
        partial_state = msg_to_state_fn(msg_transform_fn(sensor_msg), model_type)

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
    elif input_config['type'] == 'ACCEL':
        data_class = Accel
        extract_input_fn = accel_msg_to_input
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
        data_class, sensor_callback = preprocess_sensor(config['model_type'], sensor, config['solve_frame'])

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
    assert state['model_config'] is not None, "Model config should be initialized before the loop starts"
    assert state['data_pub'] is not None, "Data publisher should be initialized before the loop starts"

    # Populate the diagnostics message
    diag_msg = DiagnosticStatus()
    diag_msg.name = "Detector Update Loop"
    diag_msg.level = DiagnosticStatus.OK
    diag_msg.message = ""
    diag_msg.values = []
    add_timer_event_to_diag_status(diag_msg, event)

    if event.last_duration and event.last_duration > state['model_config']['dt']:
        diag_msg.level = max(DiagnosticStatus.WARN, diag_msg.level)
        diag_msg.message += "Last update loop took longer than the update period\n"

    if event.current_real - event.current_expected > rospy.Duration.from_sec(state['model_config']['max_update_delay'] + state['update_loop_time_debt']):
        late_sec = (event.current_real - event.current_expected).to_sec()
        late_sec_without_debt = late_sec - state['update_loop_time_debt']
        msg = f"Update loop is running late (expected at {event.current_expected.to_sec() + state['update_loop_time_debt']:.2f}, but running at {event.current_real.to_sec():.2f}, diff={late_sec_without_debt:.4f}s)! Wiping the data and starting over"
        rospy.logerr(msg)
        diag_msg.level = max(DiagnosticStatus.ERROR, diag_msg.level)
        diag_msg.message += msg + "\n"
        with state['detector_data_lock']:
            state['detector_data'] = None
        state['update_loop_time_debt'] = late_sec

    N = state['model_config']['N']
    model_type = state['model_config']['model_type']
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
            C_block = np.zeros_like(C_block)
            sensor_measurements = np.zeros(len(config['measured_states']))
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
            rospy.logerr(f"Input {input['config']['topic']} has no data. Skipping this iteration.")
            continue

        # TODO check data age

        extract_input_fn(data, u)

        inputs_present.append(config)

    with state['lock']:
        # TODO check estimate age
        estimate = state['estimate']

    if not sensors_present or not inputs_present or estimate is None:
        rospy.logerr(f"Not enough data is available for an update loop. {len(sensors_present)=}, {len(inputs_present)=}, {bool(estimate)=}")

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

        if state['detector_data'] is None:
            state['detector_data'] = DetectorData(C=[], Y=[], U=[], X=[], sensors_present=[], inputs_present=[])

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

    # Publish the current data for debugging
    publish_detector_data(state['data_pub'])

    if diag_msg.message == "":
        diag_msg.message = "ok"

    with state['diagnostics_lock']:
        state['diagnostics']['update_loop'] = diag_msg

def publish_detector_data(pub: rospy.Publisher, detector_data: Optional[DetectorData] = None):
    """
    Publish the current data for debugging
    """
    assert state['model_config'] is not None, "Model config should be initialized before this function is called"

    if detector_data is None:
        with state['detector_data_lock']:
            detector_data = deepcopy(state['detector_data'])
    
    if detector_data is None:
        rospy.loginfo("Waiting for detector data to be available before publishing")
        return

    
    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = state['model_config']['solve_frame']
    msg.poses = [state_to_pose_msg(x, state['model_config']['model_type']) for x in detector_data['X']]

    pub.publish(msg)


def solve_loop(event: rospy.timer.TimerEvent):
    """
    Solve the optimization problem to determine whether each sensor is corrupted.
    """
    assert state['model_config'] is not None, "Model config should be initialized before the loop starts"
    assert state['transform_frames'] is not None, "Transform frames should be initialized before the loop starts"
    assert state['sensor_validity_pub'] is not None, "Sensor validity publisher should be initialized before the loop starts"
    assert state['sensor_malfunction_max_magnitude_pub'] is not None, "Sensor malfunction max magnitude publisher should be initialized before the loop starts"
    assert state['solve_data_pub'] is not None, "Solve data publisher should be initialized before the loop starts"

    # Populate the diagnostics message
    diag_msg = DiagnosticStatus()
    diag_msg.name = "Detector Solve Loop"
    diag_msg.level = DiagnosticStatus.OK
    diag_msg.message = ""
    diag_msg.values = []
    add_timer_event_to_diag_status(diag_msg, event)

    # Check if the last loop took too long
    if event.last_duration and event.last_duration > DETECTOR_SOLVE_PERIOD:
        diag_msg.level = max(DiagnosticStatus.WARN, diag_msg.level)
        diag_msg.message += "Last solve loop took longer than the update period\n"

    # Get the data
    with state['detector_data_lock']:
        detector_data = deepcopy(state['detector_data'])
        path_msg_original = state['path_msg']
        transform_frames = state['transform_frames']

    if path_msg_original is None:
        rospy.loginfo("Waiting for a path message in the solve loop. Skipping this iteration.")
        return

    publish_detector_data(state['solve_data_pub'], detector_data)

    model_config = state['model_config']

    # Prepare the path
    try:
        path_msg = transform_frames.path_msg_transform(path_msg_original, model_config['solve_frame'])
    except Exception as e:
        rospy.logerr(f"Failed to transform the path to the solve frame. Skipping this solve iteration. Error: {e}")
        return
    path = Path.from_path_msg(path_msg, closed=PLANNER_PATH_CLOSED)

    # Prepare the data
    q = sum([len(s['measured_states']) for s in model_config['sensors']])
    p = len(get_model_inputs(model_config['model_type']))
    n = len(get_model_states(model_config['model_type']))
    N = model_config['N']

    if detector_data is None or len(detector_data['C']) < N:
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
    us = detector_data['U']

    measured_states = get_all_measured_states(model_config['sensors'])
    angular_outputs_mask = get_angular_mask(model_config['model_type'], measured_states)

    # Find the closest point on the path to the current state
    closest_point = path.get_closest_point(detector_data['X'][-1][:2])
    stride = VELOCITY * model_config['dt']
    # populate the desired trajectory in terms of path points
    desired_path_points = [closest_point]
    for i in range(N-1):
        desired_path_points.insert(0, path.walk(desired_path_points[0], -stride))
    desired_positions = np.array([pp.point for pp in desired_path_points])
    desired_headings = [path.get_heading_at_point(pp) for pp in desired_path_points]
    desired_velocities = [path.get_velocity_at_point(pp) for pp in desired_path_points]
    assert set(desired_velocities) == {VELOCITY}, "The desired velocities should be constant (for now)"
    desired_angular_velocities = [path.get_curvature_at_point(pp) * path.get_velocity_at_point(pp) for pp in desired_path_points]
    desired_state_trajectory = np.vstack([desired_positions.T, desired_headings, desired_velocities, desired_angular_velocities])
    desired_trajectory = (block_diag(*Cs) @ desired_state_trajectory.reshape((n*N,),order='F')).reshape((q,N), order='F')
    desired_lin_accel = [0] * N # because we currently assume constant velocity
    desired_ang_accel = [path.get_dK_ds_at_point(pp) * path.get_velocity_at_point(pp) for pp in desired_path_points] # dK/dt

    u_deviation = [u - np.array([desired_lin_accel[i], desired_ang_accel[i]]).reshape((2,1)) for i, u in enumerate(us)]

    # TODO: debug
    # rospy.logwarn(f"ud={np.vstack([desired_lin_accel, desired_ang_accel])}")
    # rospy.logwarn(f"us={np.hstack(us)}")
    # rospy.logwarn(f"du={np.hstack(u_deviation)}")

    # Linearize the model
    As = []
    Bs = []
    for i in range(N):
        A, B = linearize_model(model_config['model_type'], detector_data['X'][i], detector_data['U'][i], model_config['dt'])

        As.append(A)
        Bs.append(B)
    
    # TODO: The inputs used here need to be the deviation from the nominal inputs
    # When we linearize our model about a circular path, the nominal inputs are zero, so this is fine
    input_effects = (block_diag(*Cs) @ get_input_effect_on_state(As, Bs, u_deviation).reshape((n*N,), order='F')).reshape((q, N), order='F')

    print("Xs")
    print(np.vstack(Xs).T)

    print("desired_trajectory")
    print(desired_trajectory)

    print("input_effects")
    print(input_effects)

    corruption = np.zeros((q, N))
    # corruption[3,:] = 5

    # qxN matrix of measurements
    measurements_raw = np.vstack(Ys).T + corruption
    measurements = measurements_raw - input_effects - desired_trajectory
    measurements[angular_outputs_mask, :] = wrap_angle(measurements[angular_outputs_mask, :])

    print("measurements_raw")
    print(measurements_raw)
    print("measurements")
    print(measurements)

    Y = measurements.reshape((q*N,), order='F')
    state_evolution_matrix, Phi = get_evolution_matrices(As[:-1], Cs)

    # Solve the optimization problem
    rospy.wait_for_service("run_solver")
    try:
        # prob, x0_hat = optimize_l0(n, q, N, Phi, Y)
        run_solver = rospy.ServiceProxy("run_solver", RunSolver)
        # TODO: generate eps and sensor_protection from configuration
        # The eps values are generated from observing the maximum error for each sensor after the optimization
        sim_eps = [0.05,0.05,0.15,0.02,0.25,0.25]
        robot_circle_eps = [0.08,0.08,0.17,0.16,0.25,0.25]
        sim_figure8_eps = [1.2]
        resp = run_solver(n=n, q=q, N=N, Phi=Phi.ravel(order='F'), Y=Y, eps=sim_eps,
                          solver=RunSolverRequest.L1, max_num_corruptions=1, sensor_protection=[1,1,0,0,0,0],
                          x0_regularization_lambda=1)

        diag_msg.values.append(KeyValue(key="Solver status", value=RUN_SOLVER_RESPONSE_ENUM_TO_STR[resp.status]))

        if resp.status != RunSolverResponse.SUCCESS:
            msg = f"Solver failed: {RUN_SOLVER_RESPONSE_ENUM_TO_STR[resp.status]}"
            rospy.logerr(msg)
            diag_msg.message += msg
            diag_msg.level = max(DiagnosticStatus.ERROR, diag_msg.level)
            return
        x0_hat = np.array(resp.x0_hat)
        sensor_validity = resp.sensor_validity
        malfunction_max_magnitude = resp.malfunction_max_magnitude
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        diag_msg.message += f"Service call failed: {e}"
        diag_msg.level = max(DiagnosticStatus.ERROR, diag_msg.level)
        diag_msg.values.append(KeyValue(key="Solver status", value="Service call failed"))
        with state['diagnostics_lock']:
            state['diagnostics']['solve_loop'] = diag_msg
        return
    except Exception as e:
        rospy.logerr(f"Error in optimization: {e}")
        diag_msg.message += f"Error in optimization: {e}"
        diag_msg.level = max(DiagnosticStatus.ERROR, diag_msg.level)
        diag_msg.values.append(KeyValue(key="Solver status", value="Unknown error"))
        with state['diagnostics_lock']:
            state['diagnostics']['solve_loop'] = diag_msg
        return
    # print(prob)
    print(f"{x0_hat=}")
    print("E")
    print((Y - Phi@x0_hat).reshape((q, N), order='F'))

    sensor_validity_msg = UInt8MultiArray()
    sensor_validity_msg.data = sensor_validity
    state['sensor_validity_pub'].publish(sensor_validity_msg)
    sensor_malfunction_max_magnitude_msg = Float32MultiArray()
    sensor_malfunction_max_magnitude_msg.data = malfunction_max_magnitude
    state['sensor_malfunction_max_magnitude_pub'].publish(sensor_malfunction_max_magnitude_msg)

    rospy.logwarn(f"Solved (sensor_validity: {sensor_validity})===============================")

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
    diagarr_msg.status = []

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

    state['transform_frames'] = TransformFrames()

    # Use the current estimate of the robot's state and the planned path for linearization
    rospy.Subscriber('/odometry/global_filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PathMsg, planner_path_callback)

    state['sensor_validity_pub'] = rospy.Publisher('/bdetect/sensor_validity', UInt8MultiArray, queue_size=1)
    state['sensor_malfunction_max_magnitude_pub'] = rospy.Publisher('/bdetect/sensor_malfunction_max_magnitude', Float32MultiArray, queue_size=1)
    state['diagnostics_pub'] = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
    state['data_pub'] = rospy.Publisher('/bdetect/data', PoseArray, queue_size=1)
    state['solve_data_pub'] = rospy.Publisher('/bdetect/solve_data', PoseArray, queue_size=1)

    # Load the configuration and check its type
    config: ModelConfig = typeguard.check_type(rospy.get_param('~bdetect'), ModelConfig)
    rospy.loginfo(f"{NODE_NAME}: loaded configuration {config}")

    state['model_config'] = config

    # Create subscriptions for each sensor
    create_subscriptions(config)

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # The detector loop has 2 stages:
    # 1. Update: Gather data from the sensors and construct the C, Y, and U matrices
    rospy.Timer(rospy.Duration.from_sec(config['dt']), update_loop)
    # 2. Solve: Solve the optimization problem
    rospy.Timer(rospy.Duration.from_sec(DETECTOR_SOLVE_PERIOD), solve_loop)

    # Publish the diagnostics messages
    rospy.Timer(rospy.Duration.from_sec(min(config['dt'], DETECTOR_SOLVE_PERIOD)), diagnostics_loop)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
