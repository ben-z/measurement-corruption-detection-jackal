#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep

NODE_NAME = 'bcontrol'
RADIUS = 2 # meters
VELOCITY = 0.5 # m/s
CONTROLLER_HZ = 10 # Hz
CONTROLLER_PERIOD = 1 / CONTROLLER_HZ # seconds
# If the odometry message is older than this, it is considered invalid.
ODOM_MSG_TIMEOUT = CONTROLLER_PERIOD # seconds

state = {
    'odom_msg': None,
}

def odom_callback(odom_msg: Odometry):
    state['odom_msg'] = odom_msg

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

def tick_controller(cmd_vel_pub):
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

    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    heading = math.atan2(y, x)

    # lookahead_pose = {
        
    # }
    

    # Make the robot follow a circle of radius RADIUS meters around the origin
    # with a velocity of VELOCITY m/s
    # Hint: use the odom_msg.pose.pose.position.x and odom_msg.pose.pose.position.y
    # fields to get the current position of the robot
    # Hint: use the math.atan2() function to get the angle of the robot
    # Hint: use the pub_cmd_vel() function to publish the desired velocity
    # Hint: use the CONTROLLER_HZ variable to set the desired frequency of the controller
    # Hint: use the rospy.Rate() function to sleep for the desired period
    # Hint: use the rospy.is_shutdown() function to check if the node is shutting down
    # Hint: use the rospy.loginfo() function to print messages to the console


def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Define subscribers and publishers
    rospy.subscriber('/odometry/filtered', Odometry, odom_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # Set a rate for the publisher
    rate = rospy.Rate(CONTROLLER_HZ)

    while not rospy.is_shutdown():
        tick_controller(cmd_vel_pub)

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
