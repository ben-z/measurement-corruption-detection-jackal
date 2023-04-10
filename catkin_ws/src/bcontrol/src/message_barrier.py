#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from typing import Any
import numpy as np
from threading import Lock
from robot_localization.srv import ClearTopicData, ClearTopicDataRequest, ClearTopicDataResponse
from detector_utils import ModelConfig
from utils import typeguard
from detector import DETECTOR_SOLVE_HZ
from bcontrol.msg import SensorValidity
from copy import deepcopy

NODE_NAME = "message_barrier"

BECOME_VALID_THRESHOLD = 3 * DETECTOR_SOLVE_HZ # Number of valid messages that must be received before the sensor is considered valid again

state = {
    "pub": None,
    "sensor_validity_msg": None,
    "sensor_idx": None,
    "sensor_validity_final_pub": None,
    "sensor_validity_lock": Lock(),
    "topic_name": None,
    "detector_window_duration": None,
    "enable_ekf_rollback": None,
    "last_change_time": None,
    "cooldown_duration": None,
    "became_valid_counter": None,
}

def msg_callback(msg: Any):
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

def sensor_validity_callback(msg: SensorValidity):
    # if state['sensor_validity_msg']:
    #     # combine the two messages (take the logical AND)
    #     msg.data = [a and b for a,b in zip(msg.data, state['sensor_validity_msg'].data)]
    now = rospy.Time.now()
    with state['sensor_validity_lock']:
        old_msg = state["sensor_validity_msg"]
    # we are the only users of this state variable, so we don't need a lock
    became_valid_counter = state["became_valid_counter"]

    validity = np.array(list(msg.data), dtype=bool)

    if old_msg is None:
        old_validity = np.array([True] * len(validity), dtype=bool)
    else:
        old_validity = np.array(list(old_msg.data), dtype=bool)

    # Check to see which sensors have changed from valid to invalid
    became_invalid = np.logical_and(old_validity, np.logical_not(validity))
    became_valid_raw = np.logical_and(np.logical_not(old_validity), validity)

    # if a sensor became valid, increment the counter
    became_valid_counter = became_valid_raw + (became_valid_counter if became_valid_counter is not None else np.zeros_like(became_valid_raw, dtype=int))

    # if a sensor didn't become valid, reset the counter
    became_valid_counter[~became_valid_raw] = 0

    became_valid = became_valid_counter >= BECOME_VALID_THRESHOLD
    # reset the counter for the sensors that became valid
    became_valid_counter[became_valid] = 0
    state["became_valid_counter"] = became_valid_counter

    if np.sum(became_invalid) + np.sum(became_valid) == 0:
        rospy.logdebug("No sensor_validity change detected.")
        if state['sensor_validity_msg'] is None:
            with state['sensor_validity_lock']:
                state["sensor_validity_msg"] = msg
        return

    # Check to see if the cooldown is still active
    cooldown_secs_left = 0 if state['last_change_time'] is None else (state['last_change_time'] - now).to_sec() + state['cooldown_duration']
    if cooldown_secs_left > 0:
        rospy.logwarn(f"sensor_validity changed: {became_invalid=} {became_valid=} but cooldown is still active ({cooldown_secs_left:.2f} secs left). Ignoring change.")
        return

    rospy.logwarn(f"sensor_validity changed: {became_invalid=} {became_valid=}")
    state['last_change_time'] = now

    msg.data = validity.tolist()
    with state['sensor_validity_lock']:
        state["sensor_validity_msg"] = msg

    if state["enable_ekf_rollback"] and became_invalid[state['sensor_idx']]:
        # This sensor became invalid
        rospy.logwarn(f"sensor {state['sensor_idx']} became invalid. Performing EKF rollback")
        # Tell EKF to wipe this sensor data
        # for service in ['/ekf_localization_global/clear_topic_data', '/ekf_localization_local/clear_topic_data']:
        for service in ['/ekf_localization_global/clear_topic_data', '/ekf_localization_local/clear_topic_data']:
            rospy.logwarn(f"Calling service {service} to clear {state['topic_name']}/uncorrupted")
            rospy.wait_for_service(service)
            try:
                clear_topic_data = rospy.ServiceProxy(service, ClearTopicData)
                clear_topic_data(topic=state["topic_name"] + "/uncorrupted", starting_time=now - rospy.Duration.from_sec(state['detector_window_duration']))
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

def sensor_validity_final_pub_callback(event: rospy.timer.TimerEvent):
    with state['sensor_validity_lock']:
        sensor_validity_msg = deepcopy(state["sensor_validity_msg"])
    if sensor_validity_msg is None:
        rospy.loginfo_throttle(1.0, "No sensor_validity_msg received yet. Publishing empty message")
        sensor_validity_msg = SensorValidity()
    sensor_validity_msg.header.stamp = rospy.Time.now()
    state['sensor_validity_final_pub'].publish(sensor_validity_msg)

def main():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    rospy.loginfo(f"Node {NODE_NAME} started. Ctrl-C to stop.")

    topic_name: str = rospy.get_param("~topic_name")
    message_type: str = rospy.get_param("~message_type")
    sensor_idx: str = rospy.get_param("~sensor_idx")
    bdetect_config: ModelConfig = typeguard.check_type(rospy.get_param('~bdetect'), ModelConfig)
    
    state['topic_name'] = topic_name
    state['sensor_idx'] = int(sensor_idx)
    state['detector_window_duration'] = bdetect_config['N'] * bdetect_config['dt']
    state['enable_ekf_rollback'] = rospy.get_param('~enable_ekf_rollback')
    state['cooldown_duration'] = float(rospy.get_param('~cooldown_duration'))
    
    rospy.loginfo(f"{topic_name=} {message_type=}")

    state['sensor_validity_final_pub'] = rospy.Publisher(
        f"/{NODE_NAME}/sensor_validity_final", SensorValidity, queue_size=1)

    rospy.Subscriber(f"/{NODE_NAME}/sensor_validity_input", SensorValidity, sensor_validity_callback)
    rospy.Timer(rospy.Duration.from_sec(1.0/5.0), sensor_validity_final_pub_callback)

    if message_type == "nav_msgs/Odometry":
        state['pub'] = rospy.Publisher(topic_name + "/uncorrupted", Odometry, queue_size=1)
        rospy.Subscriber(topic_name, Odometry, msg_callback)
    elif message_type == "sensor_msgs/Imu":
        state["pub"] = rospy.Publisher(topic_name + "/uncorrupted", Imu, queue_size=1)
        rospy.Subscriber(topic_name, Imu, msg_callback)
    else:
        rospy.logerr(f"Message type {message_type} not supported")
        return

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Got ROSInterruptException. Exiting...")
