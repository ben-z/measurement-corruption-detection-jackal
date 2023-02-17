#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from time import sleep
from utils import Path, pathpoints_to_pose_array, wrap_angle, clamp, generate_circle_approximation, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample
import tf

NODE_NAME = 'bcontrol'
RADIUS = 2 # meters
VELOCITY = 0.5 # m/s
CONTROLLER_HZ = 10 # Hz
CONTROLLER_PERIOD = 1 / CONTROLLER_HZ # seconds
# If the odometry message is older than this, it is considered invalid.
ODOM_MSG_TIMEOUT = CONTROLLER_PERIOD # seconds

MAX_LINEAR_VELOCITY = 2 # m/s
MAX_ANGULAR_VELOCITY = 1 # rad/s

LOOKAHEAD_M = 2 # meters

PLANNER_PATH_CLOSED = True

state = {
    'odom_msg': None,
    'path': None,
}

def odom_callback(odom_msg: Odometry):
    state['odom_msg'] = odom_msg

def planner_path_callback(path_msg: PoseArray):
    state['path'] = Path.from_pose_array(path_msg, closed=PLANNER_PATH_CLOSED)

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
    # extract the odometry message from the state
    odom_msg = state['odom_msg']
    if odom_msg is None:
        rospy.logwarn_throttle(1, "No odometry message received yet")
        return

    # Check if the odom message is too old
    odom_msg_age = rospy.Time.now() - odom_msg.header.stamp
    if odom_msg_age > rospy.Duration(ODOM_MSG_TIMEOUT):
        rospy.logwarn(f"Odometry message is too old! Age: {odom_msg_age.to_sec()}s")
        return

    path = state['path']
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

    print(f"Current position: ({x:.2f}, {y:.2f}) m, heading: {heading:.2f} rad ({math.degrees(heading):.2f} deg)")

    # path = Path([[0,-10], [0,10], [3, 10]], closed=True)
    # path = Path(generate_figure_eight_approximation([0,0], 10, 100), closed=True)
    # path = Path(rotate_points(generate_figure_eight_approximation([0,0], 10, 100), math.pi/4), closed=True)
    # path = Path(generate_ellipse_approximation([0,0], 5, 10, 100, theta=0.5), closed=True)

    # path_points = rotate_points(generate_figure_eight_approximation([0,0], 10, 100), math.pi/4)
    # path_points_slice = lookahead_resample(path_points, [x,y], 10, 20)
    # path = Path(path_points_slice)

    # TODO: We can speed this up by doing 1 global search followed by local searches until the path gets replaced again.
    closest = path.get_closest_point([x,y])
    lookahead = path.walk(closest, LOOKAHEAD_M)

    dpos = np.array(lookahead.point) - np.array([x,y])
    dpos_norm = np.linalg.norm(dpos)
    target_heading = math.atan2(dpos[1], dpos[0])
    dheading = wrap_angle(target_heading - heading)
    Kp_heading = 1

    # Use pure pursuit to compute the desired linear and angular velocities
    linear_velocity = min(dpos_norm, MAX_LINEAR_VELOCITY)
    angular_velocity = clamp(-MAX_ANGULAR_VELOCITY, Kp_heading * dheading, MAX_ANGULAR_VELOCITY)
    
    print(f"dpos: {dpos[0]:.2f}, {dpos[1]:.2f} m, target_heading {target_heading:.2f} heading {heading:.2f} dheading: {dheading:.2f} rad ({math.degrees(dheading):.2f} deg) linvel: {linear_velocity:.2f} m/s angvel: {angular_velocity:.2f} rad/s ({math.degrees(angular_velocity):.2f} deg/s))")

    lookahead_pub.publish(pathpoints_to_pose_array([lookahead], path))
    pub_cmd_vel(cmd_vel_pub, linear_velocity, angular_velocity)
    
def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Define subscribers and publishers
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PoseArray, planner_path_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    lookahead_pub = rospy.Publisher('/bcontrol/lookahead', PoseArray, queue_size=1)

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
