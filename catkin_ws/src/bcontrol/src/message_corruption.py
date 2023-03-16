#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from typing import Any
import numpy as np

NODE_NAME = "message_corruption"

state = {
    "corruption_msg": None,
    "pub": None,
}

def odom_callback(msg: Odometry):
    assert state["pub"] is not None, "The publisher is not initialized yet"

    if state["corruption_msg"] is None:
        rospy.loginfo_throttle(1.0, "No corruption message received yet, passing through the original message")
        state['pub'].publish(msg)
        return
    
    # Combine the original message with the corruption message
    msg.pose.pose.position.x += state["corruption_msg"].pose.pose.position.x
    msg.pose.pose.position.y += state["corruption_msg"].pose.pose.position.y
    msg.pose.pose.position.z += state["corruption_msg"].pose.pose.position.z
    
    msg_orientation_rpy = np.array(tf.transformations.euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ]))
    corruption_orientation_rpy = np.array(tf.transformations.euler_from_quaternion([
        state["corruption_msg"].pose.pose.orientation.x,
        state["corruption_msg"].pose.pose.orientation.y,
        state["corruption_msg"].pose.pose.orientation.z,
        state["corruption_msg"].pose.pose.orientation.w,
    ]))

    combined_orientation_rpy = msg_orientation_rpy + corruption_orientation_rpy
    combined_orientation_quat = tf.transformations.quaternion_from_euler(*combined_orientation_rpy)
    msg.pose.pose.orientation.x = combined_orientation_quat[0]
    msg.pose.pose.orientation.y = combined_orientation_quat[1]
    msg.pose.pose.orientation.z = combined_orientation_quat[2]
    msg.pose.pose.orientation.w = combined_orientation_quat[3]

    msg.twist.twist.linear.x += state["corruption_msg"].twist.twist.linear.x
    msg.twist.twist.linear.y += state["corruption_msg"].twist.twist.linear.y
    msg.twist.twist.linear.z += state["corruption_msg"].twist.twist.linear.z
    msg.twist.twist.angular.x += state["corruption_msg"].twist.twist.angular.x
    msg.twist.twist.angular.y += state["corruption_msg"].twist.twist.angular.y
    msg.twist.twist.angular.z += state["corruption_msg"].twist.twist.angular.z

    state['pub'].publish(msg)

def corruption_callback(msg: Any):
    state["corruption_msg"] = msg

def main():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    topic_name = rospy.get_param("~topic_name")
    message_type = rospy.get_param("~message_type")
    
    rospy.loginfo(f"{topic_name=} {message_type=}")

    if message_type == "nav_msgs/Odometry":
        state['pub'] = rospy.Publisher(topic_name + "/vulnerable", Odometry, queue_size=1)
        rospy.Subscriber(topic_name, Odometry, odom_callback)
        rospy.Subscriber(topic_name + "/corruption", Odometry, corruption_callback)
    else:
        rospy.logerr(f"Message type {message_type} not supported")
        return

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
