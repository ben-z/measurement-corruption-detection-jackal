#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

NODE_NAME = 'bcontrol'

def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)

    print(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Set a rate for the publisher
    rate = rospy.Rate(10)

    # Create a Twist message
    twist_msg = Twist()

    # Set linear and angular velocities
    twist_msg.linear.x = 0.5
    twist_msg.angular.z = 0.2

    while not rospy.is_shutdown():
        # Publish the message
        cmd_vel_pub.publish(twist_msg)

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(f"Got ROSInterruptException. Exiting...")
