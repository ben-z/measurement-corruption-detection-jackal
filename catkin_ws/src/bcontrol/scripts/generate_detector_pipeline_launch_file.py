#!/usr/bin/env python3

import yaml
import argparse
from xml.etree.ElementTree import Element, SubElement, tostring, Comment
from xml.dom import minidom
import sys
import os
from pathlib import Path

# Add the src directory to the path so we can import utilities
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(f'{dir_path}/../src')

from detector_utils import sensor_type_to_msg_type
from utils import assert_and_remove_suffix


def make_extactor_code(message_type, state):
    code = ''
    if message_type == 'nav_msgs/Odometry':
        code += "m1 = nav_msgs.msg.Odometry(header=m.header, child_frame_id=m.child_frame_id);"
        code += "posecov = list(m1.pose.covariance); twistcov = list(m1.twist.covariance);"
        if state == 'X':
            code += "m1.pose.pose.position.x = m.pose.pose.position.x;"
            code += "posecov[0] = m.pose.covariance[0];"
        elif state == 'Y':
            code += "m1.pose.pose.position.y = m.pose.pose.position.y;"
            code += "posecov[7] = m.pose.covariance[7];"
        elif state == 'ORIENTATION':
            code += "m1.pose.pose.orientation = m.pose.pose.orientation;"
            # Since we are populating the orientation in all directions, we will copy the covariance for all directions
            code += "posecov[21] = m.pose.covariance[21];" # roll
            code += "posecov[28] = m.pose.covariance[28];" # pitch
            code += "posecov[35] = m.pose.covariance[35];" # yaw
        elif state == 'VELOCITY':
            code += "m1.twist.twist.linear.x = m.twist.twist.linear.x;"
            code += "twistcov[0] = m.twist.covariance[0];"
        elif state == 'ANGULAR_VELOCITY':
            code += "m1.twist.twist.angular.z = m.twist.twist.angular.z;"
            code += "twistcov[35] = m.twist.covariance[35];"
        else:
            raise Exception(f"State {state} not supported for Odometry message")
        code += "m1.pose.covariance = tuple(posecov); m1.twist.covariance = tuple(twistcov);"
    elif message_type == 'sensor_msgs/Imu':
        code += "m1 = sensor_msgs.msg.Imu(header=m.header);"
        code += "angularvelcov = list(m1.angular_velocity_covariance);"
        if state == "ANGULAR_VELOCITY":
            code += "m1.angular_velocity.z = m.angular_velocity.z;"
            code += "angularvelcov[8] = m.angular_velocity_covariance[8];"
        else:
            raise Exception(f"State {state} not supported for Imu message")
        code += "m1.angular_velocity_covariance = tuple(angularvelcov);"
    else:
        raise Exception(f"Message type {message_type} not supported")

    # Return value
    code += "m1"
    return code

def create_extractor_nodes(sensor):
    topic = sensor['topic']
    message_type = sensor_type_to_msg_type(sensor['type'])._type
    message_package = message_type.split("/")[0]
    if message_package not in ['nav_msgs', 'sensor_msgs', 'geometry_msgs']:
        raise Exception(f"Message package {message_package} not supported")

    nodes = [
        Comment("The extractor nodes split a sensor topic into multiple topics. This allows us to work with individual signals from a sensor."),
    ]

    for state in sensor['measured_states']:
        topic_out = f"{topic}/{state}"
        code = make_extactor_code(message_type, state)
        nodes.append(Element(
            "node",
            {
                "name": f"{sensor['name']}_{state.lower()}_extractor",
                "pkg": "topic_tools",
                "type": "transform",
                "required": "true",
                "args": f"{topic} {topic_out} {message_type} '{code}' --import {message_package}",
            },
        ))

    return nodes


def create_corruptor_nodes(sensor):
    topic = assert_and_remove_suffix(sensor['topic'], "/vulnerable")
    message_type = sensor_type_to_msg_type(sensor['type'])._type

    elem = Element(
        "node",
        {
            "name": f"{sensor['name']}_corruptor",
            "pkg": "bcontrol",
            "type": "message_corruptor.py",
            "required": "true",
        },
    )
    SubElement(elem, "param", {"name": "topic_name", "value": topic})
    SubElement(elem, "param", {"name": "message_type", "value": message_type})

    return [
        Comment("The corruptor node allows programmatic corruption of sensor messages"),
        elem,
    ]


def create_barrier_nodes(sensor, signal_idx_base: int):
    topic = sensor['topic']
    message_type = sensor_type_to_msg_type(sensor['type'])._type
    message_package = message_type.split("/")[0]
    if message_package not in ['nav_msgs', 'sensor_msgs', 'geometry_msgs']:
        raise Exception(f"Message package {message_package} not supported")

    nodes = [
        Comment("The barrier nodes listen to the sensor validity topic and stop passing through sensor messages if the sensor is invalid."),
    ]

    for signal_idx, state in enumerate(sensor['measured_states']):
        node = Element(
            "node",
            {
                "name": f"{sensor['name']}_{state.lower()}_barrier",
                "pkg": "bcontrol",
                "type": "message_barrier.py",
                "required": "true",
            },
        )
        SubElement(node, "param", {"name": "topic_name", "value": f"{topic}/{state}"})
        SubElement(node, "param", {"name": "message_type", "value": message_type})
        SubElement(node, "param", {"name": "sensor_idx", "value": str(signal_idx_base + signal_idx)})
        SubElement(node, "rosparam", {"command": "load", "file": "$(find bcontrol)/config/bdetect.yaml"})
        SubElement(node, "param", {"name": "enable_ekf_rollback", "value": "true"})
        SubElement(node, "param", {"name": "cooldown_duration", "value": "2.0"}) # seconds

        nodes.append(node)

    return nodes

def generate_launch_file(config, raw_config):
    launch = Element("launch")
    launch.append(Comment(f"This file is generated from the following configuration:\n{raw_config}"))

    signal_idx_base = 0

    for sensor in config["bdetect"]["sensors"]:
        launch.append(Comment(f"Sensor: {sensor['name']}"))
        launch.extend(create_corruptor_nodes(sensor))
        launch.extend(create_extractor_nodes(sensor))
        launch.extend(create_barrier_nodes(sensor, signal_idx_base))

        signal_idx_base += len(sensor['measured_states'])

    return minidom.parseString(tostring(launch)).toprettyxml(indent="  ")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ROS launch file from a bdetect YAML config.")
    parser.add_argument("input", help="YAML configuration file")
    parser.add_argument("output", help="ROS launch file")

    args = parser.parse_args()

    raw_config = Path(args.input).read_text()
    yaml_config = yaml.safe_load(raw_config)

    launch_file_content = generate_launch_file(yaml_config, raw_config)

    with open(args.output, "w") as launch_file:
        launch_file.write(launch_file_content)
