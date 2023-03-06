#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from time import sleep
from utils import Path, pathpoints_to_pose_array, wrap_angle, clamp, generate_circle_approximation, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, PathPoint
import tf2_ros
import tf
from typing import Optional, TypedDict
from threading import Lock
from planner import PLANNER_PATH_CLOSED
from transform_frames import TransformFrames

NODE_NAME = 'bcontrol'
RADIUS = 2 # meters
VELOCITY = 0.5 # m/s
CONTROLLER_HZ = 10 # Hz
CONTROLLER_PERIOD = 1 / CONTROLLER_HZ # seconds
# If the odometry message is older than this, it is considered invalid.
ODOM_MSG_TIMEOUT = CONTROLLER_PERIOD # seconds

MAX_LINEAR_VELOCITY = 0.5 # m/s
MAX_ANGULAR_VELOCITY = 1.0 # rad/s

LOOKAHEAD_M = 0.5 # meters

class State(TypedDict):
    odom_msg: Optional[Odometry]
    path: Optional[Path]
    closest_path_point: Optional[PathPoint]
    transform_frames: Optional[TransformFrames]
    lock: Lock

state: State = {
    'odom_msg': None,
    'path': None,
    'closest_path_point': None,
    'transform_frames': None,
    'lock': Lock(),
}

def odom_callback(odom_msg: Odometry):
    transform_frames = state["transform_frames"]
    if transform_frames is None:
        rospy.logerr("No transform listener available. Cannot transform the odometry message to the odom frame.")
        return

    # Transform the odometry message to the map frame
    try: 
        odom_msg_map = transform_frames.odom_transform(odom_msg, target_frame="map")
    except Exception as e:
        rospy.logerr(f"Error transforming the odometry message to the odom frame: {e}")
        return

    with state['lock']:
        state['odom_msg'] = odom_msg_map

def planner_path_callback(path_msg: PoseArray):
    new_path = Path.from_pose_array(path_msg, closed=PLANNER_PATH_CLOSED)
    with state['lock']:
        if new_path == state['path']:
            rospy.logdebug("Received the same path as before. Ignoring it.")
            return
        state['path'] = new_path
        state['closest_path_point'] = None # reset the closest path point now that we have a new path

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

def tick_controller(cmd_vel_pub, lookahead_pub):
    # Extract the state variables atomically
    # Note: we never mutate state variables (only replace them) so we don't need to
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
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y

    orientation = odom_msg.pose.pose.orientation
    # Convert the orientation quaternion to a yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, heading = tf.transformations.euler_from_quaternion(q)

    rospy.logdebug(f"Current position: ({x:.2f}, {y:.2f}) m, heading: {heading:.2f} rad ({math.degrees(heading):.2f} deg)")

    if state['closest_path_point'] is None:
        closest = path.get_closest_point([x,y])
    else:
        closest = path.get_local_closest_point([x,y], state['closest_path_point'])
    state['closest_path_point'] = closest

    lookahead = path.walk(closest, LOOKAHEAD_M)

    dpos = np.array(lookahead.point) - np.array([x,y])
    dpos_norm = np.linalg.norm(dpos)
    target_heading = math.atan2(dpos[1], dpos[0])
    dheading = wrap_angle(target_heading - heading)
    Kp_pos = 1.0
    Kp_heading = 2.0

    # Use pure pursuit to compute the desired linear and angular velocities
    linear_velocity = min(Kp_pos * dpos_norm, MAX_LINEAR_VELOCITY)
    angular_velocity = clamp(Kp_heading * dheading, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
    
    rospy.logdebug(f"dpos: {dpos[0]:.2f}, {dpos[1]:.2f} m, target_heading {target_heading:.2f} heading {heading:.2f} dheading: {dheading:.2f} rad ({math.degrees(dheading):.2f} deg) linvel: {linear_velocity:.2f} m/s angvel: {angular_velocity:.2f} rad/s ({math.degrees(angular_velocity):.2f} deg/s))")

    lookahead_pub.publish(pathpoints_to_pose_array([lookahead], path, frame_id="map"))
    pub_cmd_vel(cmd_vel_pub, linear_velocity, angular_velocity)
    
def main():
    # Initialize the node
    rospy.init_node(NODE_NAME,log_level=rospy.DEBUG)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Define subscribers and publishers
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PoseArray, planner_path_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    lookahead_pub = rospy.Publisher('/bcontrol/lookahead', PoseArray, queue_size=1)

    state["transform_frames"] = TransformFrames()

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # Set a rate for the publisher
    rate = rospy.Rate(CONTROLLER_HZ)

    while not rospy.is_shutdown():
        tick_controller(cmd_vel_pub, lookahead_pub)

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
