#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray, Accel
from nav_msgs.msg import Odometry
import numpy as np
from time import sleep
from utils import Path, pathpoints_to_pose_array, wrap_angle, clamp, generate_circle_approximation, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, PathPoint
import tf2_ros
import tf
from typing import Optional, TypedDict, Tuple
from threading import Lock
from planner import PLANNER_PATH_CLOSED
from transform_frames import TransformFrames
from bcontrol.msg import Path as PathMsg
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from bcontrol.cfg import BControllerConfig
from controller_utils import drive_to_target_point


NODE_NAME = 'bcontrol'
RADIUS = 2 # meters
CONTROLLER_HZ = 10 # Hz
CONTROLLER_PERIOD = 1 / CONTROLLER_HZ # seconds
CONTROLLER_PUBLISH_HZ = 100 # Hz
# If the odometry message is older than this, it is considered invalid.
ODOM_MSG_TIMEOUT = CONTROLLER_PERIOD # seconds

# How often to transform the path to the odom frame
PATH_TRANSFORMATION_HZ = 10 # Hz
PATH_TRANSFORMATION_PERIOD = 1 / PATH_TRANSFORMATION_HZ # seconds

# Another layer of limits are set in the configuration file for jackal_control
MAX_LINEAR_VELOCITY = 1.0 # m/s
MAX_ANGULAR_VELOCITY = 2.0 # rad/s
MAX_LINEAR_ACCELERATION = 2.0 # m/s^2
MAX_ANGULAR_ACCELERATION = 25.0 # rad/s^2

class State(TypedDict):
    odom_msg: Optional[Odometry]
    path_id: int
    transformed_path_id: int
    path_msg: Optional[PathMsg]
    path: Optional[Path]
    closest_path_point: Optional[PathPoint]
    transform_frames: Optional[TransformFrames]
    lock: Lock
    cmd_vel: Tuple[float,float] # (linear, angular)
    cmd_accel: Optional[Tuple[float,float]] # (linear, angular)
    cmd_vel_pub: Optional[rospy.Publisher]
    cmd_accel_pub: Optional[rospy.Publisher]
    Kp_feedforward: Optional[float]
    Kp_heading: Optional[float]
    Kp_lateral_position: Optional[float]
    Kp_angvel: Optional[float]
    Kp_vel: Optional[float]
    lookahead_m: Optional[float]

state: State = {
    'odom_msg': None,
    'path_id': -1,
    'transformed_path_id': -1,
    'path_msg': None,
    'path': None,
    'closest_path_point': None,
    'transform_frames': None,
    'lock': Lock(),
    'cmd_vel': (0,0),
    'cmd_accel': None,
    'cmd_vel_pub': None,
    'cmd_accel_pub': None,
    'Kp_feedforward': None,
    'Kp_heading': None,
    'Kp_lateral_position': None,
    'Kp_angvel': None,
    'Kp_vel': None,
    'lookahead_m': None,
}

def odom_callback(odom_msg: Odometry):
    with state['lock']:
        state['odom_msg'] = odom_msg

def planner_path_callback(path_msg: PathMsg):
    with state['lock']:
        if state['path_msg'] and path_msg.poses == state['path_msg'].poses:
            rospy.logdebug("Received the same path as before. Ignoring it.")
            return
        state['path_msg'] = path_msg
        
def transform_path_callback(event: rospy.timer.TimerEvent):
    with state['lock']:
        path_id = state['path_id']
        transformed_path_id = state['transformed_path_id']
        path_msg = state['path_msg']
        transform_frames = state['transform_frames']

    assert transform_frames is not None, "No transform listener available. Cannot transform the path to the odom frame."

    if path_msg is None:
        rospy.logwarn_throttle(1, "No path received yet. Not transforming it.")
        return

    # Transform the path to the odom frame
    try:
        path_msg_odom = transform_frames.path_msg_transform(path_msg, target_frame="odom")
    except Exception as e:
        rospy.logerr(f"Error transforming the path to the odom frame: {e}")
        return

    new_path = Path.from_path_msg(path_msg_odom, closed=PLANNER_PATH_CLOSED)

    with state['lock']:
        state['path'] = new_path
    
        if path_id != transformed_path_id:
            rospy.loginfo("Path updated. Resetting the closest path point.")
            state['closest_path_point'] = None # reset the closest path point now that we have a new path
            state['transformed_path_id'] = path_id


def publish_cmd_vel_callback(event: rospy.timer.TimerEvent):
    with state['lock']:
        cmd_vel = state['cmd_vel']
        cmd_accel = state['cmd_accel']
        cmd_vel_pub = state['cmd_vel_pub']
        cmd_accel_pub = state['cmd_accel_pub']
    
    assert cmd_vel_pub is not None, "cmd_vel_pub should already be initialized."
    assert cmd_accel_pub is not None, "cmd_accel_pub should already be initialized."
    
    if cmd_accel is None:
        rospy.logwarn_throttle(1, "No acceleration command available. Not publishing the velocity.")
        return
    
    cmd_vel = (
        clamp(cmd_vel[0] + cmd_accel[0] * (1.0/CONTROLLER_PUBLISH_HZ), -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY),
        clamp(cmd_vel[1] + cmd_accel[1] * (1.0/CONTROLLER_PUBLISH_HZ), -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY),
    )
    
    accel_msg = Accel()
    accel_msg.linear.x = cmd_accel[0]
    accel_msg.angular.z = cmd_accel[1]
    cmd_accel_pub.publish(accel_msg)
    
    # Publish the velocity command
    pub_cmd_vel(cmd_vel_pub, *cmd_vel)


    with state['lock']:
        state['cmd_vel'] = cmd_vel
    

def pub_cmd_vel(cmd_vel_pub, linear_vel, angular_vel):
    """
    Publishes a Twist message to the /cmd_vel topic.
    Arguments:
        cmd_vel_pub: The publisher object
        linear_vel: The linear velocity in m/s
        angular_vel: The angular velocity in rad/s
    """
    # Create a Twist message
    twist_msg = Twist()

    # Set linear and angular velocities
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel

    # Publish the message
    cmd_vel_pub.publish(twist_msg)


def tick_dynamics_controller(
    lookahead_pub,
    lateral_position_pub,
    target_heading_pub,
    target_angular_velocity_pub,
    actual_heading_pub,
    angular_accel_feedforward_pub,
    angular_accel_angvel_pub,
    angular_accel_heading_pub,
    angular_accel_latdev_pub,  
):
    # Extract the state variables atomically
    # ote: we never mutate state variables (only replace them) so we don't need to
    # worry about the state changing while we're using it.
    with state['lock']:
        odom_msg = state['odom_msg']
        path = state['path']
        cmd_accel_pub = state['cmd_accel_pub']
        cmd_vel = state['cmd_vel']
        Kp_feedforward = state['Kp_feedforward']
        Kp_heading = state['Kp_heading']
        Kp_lateral_position = state['Kp_lateral_position']
        Kp_angvel = state['Kp_angvel']
        Kp_vel = state['Kp_vel']
        lookahead_m = state['lookahead_m']

    if Kp_feedforward is None \
        or Kp_heading is None \
        or Kp_lateral_position is None \
        or Kp_angvel is None \
        or Kp_vel is None \
        or lookahead_m is None \
        :
        rospy.logwarn_throttle(1, "Controller parameters not yet received. Not updating the controller.")
        return

    assert cmd_accel_pub is not None, "cmd_accel_pub should already be initialized."

    if odom_msg is None:
        rospy.logwarn_throttle(1, "No odometry message received yet")
        return

    # Check if the odom message is too old
    odom_msg_age = rospy.Time.now() - odom_msg.header.stamp
    if odom_msg_age > rospy.Duration(ODOM_MSG_TIMEOUT):
        rospy.logwarn(f"Odometry message is too old! Age: {odom_msg_age.to_sec()}s. Not updating the controller.")
        return

    if path is None:
        rospy.logwarn_throttle(1, "No path received yet")
        return

    # Get the current position and heading estimates of the robot
    x: float = odom_msg.pose.pose.position.x
    y: float = odom_msg.pose.pose.position.y
    v: float = odom_msg.twist.twist.linear.x
    omega: float = odom_msg.twist.twist.angular.z # yaw rate
    orientation = odom_msg.pose.pose.orientation
    # Convert the orientation quaternion to a yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll: float
    pitch: float
    heading: float
    roll, pitch, heading = tf.transformations.euler_from_quaternion(q)
    actual_heading_pub.publish(heading)

    rospy.logdebug(f"Current position: ({x:.2f}, {y:.2f}) m, heading: {heading:.2f} rad ({math.degrees(heading):.2f} deg), v: {v:.2f} m/s, omega: {omega:.2f} rad/s")

    if state['closest_path_point'] is None:
        # Global search, slow
        closest = path.get_closest_point([x,y])
    else:
        # Local search, fast
        closest = path.get_local_closest_point([x,y], state['closest_path_point'])
    state['closest_path_point'] = closest

    lookahead = path.walk(closest, lookahead_m)
    lookahead_pub.publish(pathpoints_to_pose_array([lookahead], path, frame_id="odom"))

    target_heading = path.get_heading_at_point(closest)
    target_v = path.get_velocity_at_point(closest) # ds/dt
    target_curvature = path.get_curvature_at_point(lookahead) # dK/ds
    target_ang_vel = target_curvature * target_v # dK/dt

    # Kp_feedforward = 0.0#1.0
    # Calculate feedforward angular acceleration
    dK_ds = path.get_dK_ds_at_point(closest)
    dK_dt = dK_ds * target_v
    feedforward_contrib = Kp_feedforward * dK_dt
    angular_accel_feedforward_pub.publish(feedforward_contrib)

    # Use feedback to compensate for heading/position/curvature error
    # Kp_heading = 1.0#5.0 #2.0
    # Kp_lateral_position = 2.0#4.0 #4.0
    # Kp_angvel = 1.0#5.0 # 2.0
    heading_error = wrap_angle(target_heading - heading)
    target_heading_pub.publish(target_heading)
    lateral_position = path.get_lateral_position([x,y], closest)
    lateral_position_pub.publish(lateral_position)
    lateral_error = -lateral_position
    angvel_error = target_ang_vel - omega
    target_angular_velocity_pub.publish(target_ang_vel)

    if abs(lateral_error) > 1.0 or abs(heading_error) > math.radians(90):
        # Fall back to pure pursuit
        rospy.logwarn(f"Lateral error ({lateral_error:.2f} m) or heading error ({math.degrees(heading_error):.2f} deg) too large. Falling back to pure pursuit.")

        assert closest.point is not None, "closest.point should not be None"
        linear_accel_cmd, angular_accel_cmd = drive_to_target_point([x, y, heading], v, omega, [closest.point[0], closest.point[1], target_heading], 0.5, MAX_LINEAR_ACCELERATION, 1.0, MAX_ANGULAR_ACCELERATION)
    
        with state['lock']:
            state['cmd_accel'] = (linear_accel_cmd, angular_accel_cmd)
        return

    heading_contrib = Kp_heading * heading_error
    angular_accel_heading_pub.publish(heading_contrib)
    latdev_contrib = Kp_lateral_position * lateral_error
    angular_accel_latdev_pub.publish(latdev_contrib)
    angvel_contrib = Kp_angvel * angvel_error
    angular_accel_angvel_pub.publish(angvel_contrib)

    angular_accel_cmd = clamp(feedforward_contrib + heading_contrib + latdev_contrib + angvel_contrib, -MAX_ANGULAR_ACCELERATION, MAX_ANGULAR_ACCELERATION)

    # Use feedback to compensate for velocity error
    dvel = target_v - v
    # Kp_vel = 1.0
    linear_accel_cmd = clamp(Kp_vel * dvel, -MAX_LINEAR_ACCELERATION, MAX_LINEAR_ACCELERATION)

    with state['lock']:
        state['cmd_accel'] = (linear_accel_cmd, angular_accel_cmd)

def tick_pure_pursuit(lookahead_pub, cmd_vel_pub):
    # Extract the state variables atomically
    # ote: we never mutate state variables (only replace them) so we don't need to
    # worry about the state changing while we're using it.
    with state['lock']:
        odom_msg = state['odom_msg']
        path = state['path']

    if odom_msg is None:
        rospy.logwarn_throttle(1, "No odometry message received yet")
        return

    # Check if the odom message is too old
    odom_msg_age = rospy.Time.now() - odom_msg.header.stamp
    if odom_msg_age > rospy.Duration(ODOM_MSG_TIMEOUT):
        rospy.logwarn(f"Odometry message is too old! Age: {odom_msg_age.to_sec()}s. Not updating the controller.")
        return

    if path is None:
        rospy.logwarn_throttle(1, "No path received yet")
        return

    # Get the current position and heading estimates of the robot
    x: float = odom_msg.pose.pose.position.x
    y: float = odom_msg.pose.pose.position.y
    orientation = odom_msg.pose.pose.orientation
    # Convert the orientation quaternion to a yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll: float
    pitch: float
    heading: float
    roll, pitch, heading = tf.transformations.euler_from_quaternion(q)

    if state['closest_path_point'] is None:
        # Global search, slow
        closest = path.get_closest_point([x,y])
    else:
        # Local search, fast
        closest = path.get_local_closest_point([x,y], state['closest_path_point'])

    state['closest_path_point'] = closest

    lookahead = path.walk(closest, 0.5)
    target_heading = path.get_heading_at_point(lookahead)

    dpos = np.array(lookahead.point) - np.array([x,y])
    dpos_norm = np.linalg.norm(dpos)
    target_heading = math.atan2(dpos[1], dpos[0])
    dheading = wrap_angle(target_heading - heading)
    Kp_pos = 1.0
    Kp_heading = 3.0

    # Use pure pursuit to compute the desired linear and angular velocities
    linear_velocity = min(Kp_pos * dpos_norm, MAX_LINEAR_VELOCITY)
    angular_velocity = clamp(Kp_heading * dheading, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
    
    rospy.logdebug(f"dpos: {dpos[0]:.2f}, {dpos[1]:.2f} m, target_heading {target_heading:.2f} heading {heading:.2f} dheading: {dheading:.2f} rad ({math.degrees(dheading):.2f} deg) linvel: {linear_velocity:.2f} m/s angvel: {angular_velocity:.2f} rad/s ({math.degrees(angular_velocity):.2f} deg/s))")

    lookahead_pub.publish(pathpoints_to_pose_array([lookahead], path, frame_id="odom"))
    pub_cmd_vel(cmd_vel_pub, linear_velocity, angular_velocity)
    
def reconfigure_callback(config: BControllerConfig, level):
    rospy.logwarn(f"Reconfigure request ({level=}): {config}")

    with state['lock']:
        state["Kp_feedforward"] = config.Kp_feedforward
        state["Kp_heading"] = config.Kp_heading
        state["Kp_lateral_position"] = config.Kp_lateral_position
        state["Kp_angvel"] = config.Kp_angvel
        state["Kp_vel"] = config.Kp_vel
        state['lookahead_m'] = config.lookahead_m
    
    return config
    
def main():
    # Initialize the node
    rospy.init_node(NODE_NAME,log_level=rospy.DEBUG)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    state["transform_frames"] = TransformFrames()

    # Wait for a few seconds for the upstream nodes to start
    sleep(6)

    # Define subscribers and publishers
    rospy.Subscriber('/odometry/local_filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PathMsg, planner_path_callback)
    rospy.Timer(rospy.Duration.from_sec(PATH_TRANSFORMATION_PERIOD), transform_path_callback)
    rospy.Timer(rospy.Duration.from_sec(1.0/CONTROLLER_PUBLISH_HZ), publish_cmd_vel_callback)
    state['cmd_vel_pub'] = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    state['cmd_accel_pub'] = rospy.Publisher('/bcontrol/cmd_accel', Accel, queue_size=1)
    lookahead_pub = rospy.Publisher('/bcontrol/lookahead', PoseArray, queue_size=1)
    lateral_position_pub = rospy.Publisher('/bcontrol/lateral_position', Float64, queue_size=1)
    target_heading_pub = rospy.Publisher('/bcontrol/target_heading', Float64, queue_size=1)
    actual_heading_pub = rospy.Publisher('/bcontrol/actual_heading', Float64, queue_size=1)
    target_angular_velocity_pub = rospy.Publisher('/bcontrol/target_angular_velocity', Float64, queue_size=1)
    angular_accel_feedforward_pub = rospy.Publisher('/bcontrol/angular_accel_components/feedforward', Float64, queue_size=1)
    angular_accel_angvel_pub = rospy.Publisher('/bcontrol/angular_accel_components/angvel', Float64, queue_size=1)
    angular_accel_heading_pub = rospy.Publisher('/bcontrol/angular_accel_components/heading', Float64, queue_size=1)
    angular_accel_latdev_pub = rospy.Publisher('/bcontrol/angular_accel_components/latdev', Float64, queue_size=1)

    reconfigure_server = DynamicReconfigureServer(BControllerConfig, reconfigure_callback)

    # Wait for a few seconds for data to start coming in
    # This is not required and is only for reducing warnings
    # at startup
    sleep(1)

    # Set a rate for the publisher
    rate = rospy.Rate(CONTROLLER_HZ)

    while not rospy.is_shutdown():
        tick_pure_pursuit(lookahead_pub, state['cmd_vel_pub'])
        # tick_dynamics_controller(
        #     lookahead_pub,
        #     lateral_position_pub,
        #     target_heading_pub,
        #     target_angular_velocity_pub,
        #     actual_heading_pub,
        #     angular_accel_feedforward_pub,
        #     angular_accel_angvel_pub,
        #     angular_accel_heading_pub,
        #     angular_accel_latdev_pub,
        # )

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
