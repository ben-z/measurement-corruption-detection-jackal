#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseArray
from time import sleep
from utils import Path, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample

NODE_NAME = 'bplan'
RADIUS = 2 # meters
VELOCITY = 0.5 # m/s
PLANNER_HZ = 1 # Hz
PLANNER_PERIOD = 1 / PLANNER_HZ # seconds
  
def tick_planner(path_pub):
    path = Path(rotate_points(generate_figure_eight_approximation([0,0], 10, 100), math.pi/4), closed=True)

    path_msg = path.to_pose_array()
    path_pub.publish(path_msg)
  
def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Define subscribers and publishers
    path_pub = rospy.Publisher('/bplan/path', PoseArray, queue_size=1)

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # Set a rate for the publisher
    rate = rospy.Rate(PLANNER_HZ)

    while not rospy.is_shutdown():
        tick_planner(path_pub)

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
