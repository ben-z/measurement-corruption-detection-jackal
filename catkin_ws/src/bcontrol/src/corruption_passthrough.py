#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from typing import Any
import numpy as np
from std_msgs.msg import UInt8MultiArray

NODE_NAME = "corruption_passthrough"

state = {
    "corruption_msg": None,
    "pub": None,
    "sensor_validity_msg": None,
    "sensor_idx": None,
}

def odom_callback(msg: Odometry):
    assert state["pub"] is not None, "The publisher is not initialized yet"
    assert state["sensor_idx"] is not None, "The sensor_idx must be set"

    if state["sensor_validity_msg"] is None:
        rospy.loginfo_throttle(1.0, "No sensor_validity_msg received yet. Passing through")
        state['pub'].publish(msg)
        return
    
    sensor_validity = state['sensor_validity_msg'].data
    if sensor_validity[state['sensor_idx']]:
        state['pub'].publish(msg)
    else:
        rospy.loginfo_throttle(1.0, f"sensor {state['sensor_idx']} is corrupted, not passing through.")

def sensor_validity_callback(msg: UInt8MultiArray):
    # if state['sensor_validity_msg']:
    #     # combine the two messages (take the logical AND)
    #     msg.data = [a and b for a,b in zip(msg.data, state['sensor_validity_msg'].data)]

    validity = list(msg.data)
    # assume angular velocity is good (HACK)
    # validity[4] = 1
    msg.data = validity
    rospy.logwarn(f"sensor_validity_msg: {msg.data}")
    state["sensor_validity_msg"] = msg

def main():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    topic_name: str = rospy.get_param("~topic_name")
    message_type: str = rospy.get_param("~message_type")
    sensor_idx: str = rospy.get_param("~sensor_idx")
    
    state['sensor_idx'] = int(sensor_idx)
    
    rospy.loginfo(f"{topic_name=} {message_type=}")

    rospy.Subscriber("/bdetect/sensor_validity", UInt8MultiArray, sensor_validity_callback)

    if message_type == "nav_msgs/Odometry":
        state['pub'] = rospy.Publisher(topic_name + "/uncorrupted", Odometry, queue_size=1)
        rospy.Subscriber(topic_name + "/vulnerable", Odometry, odom_callback)
    else:
        rospy.logerr(f"Message type {message_type} not supported")
        return

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
