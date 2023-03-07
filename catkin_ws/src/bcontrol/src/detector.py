#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from time import sleep
from utils import Path, generate_figure_eight_approximation, generate_ellipse_approximation, rotate_points, lookahead_resample, typeguard
from typing import Optional, TypedDict
from threading import Lock
from planner import PLANNER_PATH_CLOSED
from enum import Enum
from detector_utils import Config

NODE_NAME = 'bdetect'
DETECTOR_HZ = 1 # Hz
DETECTOR_PERIOD = 1 / DETECTOR_HZ # seconds

# TODO: Implement the detector
# Inputs: odom from IMU, x, y, orientation from GPS, path from the planner
# Outputs:
# - Whether each sensor is corrupted
# - Passthrough of sensors that are not corrupted

class State(TypedDict):
    estimate: Optional[Odometry]
    path: Optional[Path]
    lock: Lock

state: State = {
    'estimate': None,
    'path': None,
    'lock': Lock(),
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

def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)

    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    # Use the current estimate of the robot's state and the planned path for linearization
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rospy.Subscriber('/bplan/path', PoseArray, planner_path_callback)

    cmd_vel_pub = rospy.Publisher('/bdetect/sensor_validity', UInt8MultiArray, queue_size=1)

    # Load the configuration and check its type
    config: Config = typeguard.check_type(rospy.get_param('~bdetect'), Config)
    rospy.loginfo(f"{NODE_NAME}: loaded configuration {config}")

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # Set a rate for the publisher
    rate = rospy.Rate(DETECTOR_HZ)

    while not rospy.is_shutdown():
        tick_detector()

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
