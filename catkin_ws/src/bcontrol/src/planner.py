#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseArray
from bcontrol.msg import Path as PathMsg
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

def str_to_obj(input_str):
    # Initialize an empty dictionary to store the key-value pairs
    obj = {}
    
    # Split the input string at each comma to get the key-value pairs
    pairs = input_str.split(',')
    
    # Iterate over each key-value pair
    for pair in pairs:
        # Split the pair at the equal sign to separate the key and value
        key, value = pair.split('=')
        
        # Store the key-value pair in the dictionary
        obj[key] = value
        
    return obj

def generate_path(path_profile):
    num_points = int(path_profile.get("num_points", 100))

    if path_profile.get("type") == "circle":
        assert "radius" in path_profile, "Circle path profile must have a radius"
        radius = float(path_profile["radius"])

        points, headings, curvatures, dK_ds_list = generate_circle_approximation([0,0], radius, num_points)
    elif path_profile.get("type") == "figure_eight":
        assert "length" in path_profile, "Figure 8 path profile must have a length"
        assert "width" in path_profile, "Figure 8 path profile must have a width"
        length = float(path_profile["length"])
        width = float(path_profile["width"])

        points, headings, curvatures, dK_ds_list = generate_figure_eight_approximation([0,0], length, width, num_points)
    else:
        raise ValueError(f"Unknown path profile type: {path_profile.get('type')}")

    path = Path(points, headings, curvatures, dK_ds_list, velocities=[VELOCITY]*len(points), closed=PLANNER_BASE_PATH_CLOSED)
    return path


def tick_planner(path_pub, pose_array_pub, path_profile):
    # Figure 8
    # points, headings, curvatures, dK_ds_list = generate_figure_eight_approximation([0,0], 4, 2, 100)
    # points, headings, curvatures, dK_ds_list = generate_figure_eight_approximation([0,0], 5, 2.6, 100)
    # points, headings, curvatures, dK_ds_list = generate_figure_eight_approximation([0,0], 10, 5, 100)
    # path = Path(points, headings, curvatures, dK_ds_list, velocities=[VELOCITY]*len(points), closed=PLANNER_BASE_PATH_CLOSED)

    # Circle
    # points, headings, curvatures, dK_ds_list = generate_circle_approximation([0,0], RADIUS, 100)
    # path = Path(points, headings, curvatures, dK_ds_list, velocities=[VELOCITY]*len(points), closed=PLANNER_BASE_PATH_CLOSED)

    path = generate_path(path_profile)

    path_pub.publish(path.to_path_msg())
    pose_array_pub.publish(path.to_pose_array())
  
def main():
    # Initialize the node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    path_profile = str_to_obj(rospy.get_param("~path_profile", "type=circle,radius=2"))

    # Define subscribers and publishers
    path_pub = rospy.Publisher('/bplan/path', PathMsg, queue_size=1)
    # for visualization
    pose_array_pub = rospy.Publisher('/bplan/path/pose_array', PoseArray, queue_size=1)

    # Wait for a few seconds for the upstream nodes to start
    sleep(3)

    # Set a rate for the publisher
    rate = rospy.Rate(PLANNER_HZ)

    while not rospy.is_shutdown():
        tick_planner(path_pub, pose_array_pub, path_profile)

        # Sleep for the desired period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
