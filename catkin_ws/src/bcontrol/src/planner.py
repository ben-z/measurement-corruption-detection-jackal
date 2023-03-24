#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseArray
from time import sleep
from utils import Path, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, generate_circle_approximation

NODE_NAME = 'bplan'
RADIUS = 2 # meters
VELOCITY = 0.5 # m/s
PLANNER_HZ = 1 # Hz
PLANNER_PERIOD = 1 / PLANNER_HZ # seconds
# Whether the underlying reference path of the planner is closed
PLANNER_BASE_PATH_CLOSED = True
# Whether the path published by the planner is closed
PLANNER_PATH_CLOSED = PLANNER_BASE_PATH_CLOSED
  
def tick_planner(path_pub):
    # path = Path(rotate_points(generate_figure_eight_approximation([0,0], 2, 100), 0), closed=PLANNER_BASE_PATH_CLOSED)
    path = Path(generate_circle_approximation([0,0], 2, 100), closed=PLANNER_BASE_PATH_CLOSED)

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
