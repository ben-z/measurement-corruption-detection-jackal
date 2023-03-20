#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from typing import Any
import numpy as np
from std_msgs.msg import UInt8MultiArray
from threading import Lock

NODE_NAME = "corruption_passthrough"

state = {
    "corruption_msg": None,
    "pub": None,
    "sensor_validity_msg": None,
    "sensor_idx": None,
    "sensor_validity_final_pub": None,
    "sensor_validity_lock": Lock(),
}

def odom_callback(msg: Odometry):
    assert state["pub"] is not None, "The publisher is not initialized yet"
    assert state["sensor_idx"] is not None, "The sensor_idx must be set"

    with state['sensor_validity_lock']:
        sensor_validity_msg = state["sensor_validity_msg"]

    if sensor_validity_msg is None:
        rospy.loginfo_throttle(1.0, "No sensor_validity_msg received yet. Passing through")
        state['pub'].publish(msg)
        return
    
    sensor_validity = sensor_validity_msg.data
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
    with state['sensor_validity_lock']:
        state["sensor_validity_msg"] = msg

def sensor_validity_final_pub_callback(event: rospy.timer.TimerEvent):
    with state['sensor_validity_lock']:
        sensor_validity_msg = state["sensor_validity_msg"]
    if sensor_validity_msg is None:
        rospy.loginfo_throttle(1.0, "No sensor_validity_msg received yet. Publishing empty message")
        sensor_validity_msg = UInt8MultiArray()
    state['sensor_validity_final_pub'].publish(sensor_validity_msg)

def main():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    topic_name: str = rospy.get_param("~topic_name")
    message_type: str = rospy.get_param("~message_type")
    sensor_idx: str = rospy.get_param("~sensor_idx")
    
    state['sensor_idx'] = int(sensor_idx)
    
    rospy.loginfo(f"{topic_name=} {message_type=}")

    state['sensor_validity_final_pub'] = rospy.Publisher("/corruption_passthrough/sensor_validity_final", UInt8MultiArray, queue_size=1)

    rospy.Subscriber("/bdetect/sensor_validity", UInt8MultiArray, sensor_validity_callback)
    rospy.Timer(rospy.Duration(1), sensor_validity_final_pub_callback)

    if message_type == "nav_msgs/Odometry":
        state['pub'] = rospy.Publisher(topic_name + "/uncorrupted", Odometry, queue_size=1)
        rospy.Subscriber(topic_name, Odometry, odom_callback)
    else:
        rospy.logerr(f"Message type {message_type} not supported")
        return

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
