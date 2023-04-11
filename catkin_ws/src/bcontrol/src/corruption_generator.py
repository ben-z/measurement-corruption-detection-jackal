#!/usr/bin/env python3

import rospy
import numpy as np
import argparse
from signal_generator import corrupt_array, SIGNAL_TYPE_STEP, SIGNAL_TYPE_RAMP, SIGNAL_TYPE_OSCILLATING
from numpy_message_converter import OdometryConverter, ImuConverter
from ros_utils import load_message_type
import time
from copy import deepcopy
import sys
from bcontrol.msg import Metadata
import json

# Define message converters for supported message types
MESSAGE_CONVERTERS = {
    'nav_msgs/Odometry': OdometryConverter,
    'sensor_msgs/Imu': ImuConverter,
}

class SignalGeneratorNode:
    def __init__(self, output_topic, message_type, field, signal_type, magnitude, period, corruption_start_sec, on_start=None, on_stop=None):
        self.signal_type = signal_type
        self.magnitude = magnitude
        self.period = period
        self.called_start_callback = False
        self.on_start = on_start
        self.on_stop = on_stop
        self.shutting_down = False

        # Determine the message converter based on the message type
        if message_type in MESSAGE_CONVERTERS:
            self.message_converter = MESSAGE_CONVERTERS[message_type]()
        else:
            raise ValueError(f"Unsupported message type '{message_type}'")

        # Get indices for the specified field
        self.indices = self.message_converter.get_indices(field)
        self.corruption_specs = [{'signal_type': self.signal_type,
                                  'magnitude': self.magnitude, 'indices': self.indices, 'period': self.period}]

        # Resolve message type and create publisher
        MessageType = load_message_type(message_type)
        self.corrupted_pub = rospy.Publisher(
            output_topic, MessageType, queue_size=10)

        # Create message template
        self.message_type = MessageType

        self.init_time = rospy.get_time() + corruption_start_sec

    def generate_and_publish_signal(self, event):
        if self.shutting_down:
            return
        t = rospy.get_time() - self.init_time
        if t < 0:
            rospy.logwarn_throttle(1.0, f"Corruption starting in {-t:.2f} secs.")
            self.publish_msg(self.message_type())
            return
        np_data = self.message_converter.to_numpy(self.message_type())
        corrupted_data = corrupt_array(np_data, self.corruption_specs, t)
        corrupted_msg = self.message_converter.from_numpy(
            corrupted_data, self.message_type())
        
        rospy.logwarn_once("Publishing corrupted message.")
        self.publish_msg(corrupted_msg)
        
        if not self.called_start_callback:
            if self.on_start is not None:
                self.on_start(self)
            self.called_start_callback = True

    def publish_msg(self, msg):
        # If a header is present, set the timestamp
        if hasattr(msg, 'header'):
            msg.header.stamp = rospy.Time.now()

        self.corrupted_pub.publish(msg)

    def on_shutdown(self):
        # Publish the template message when shutting down
        rospy.logwarn("Shutting down, clearing corruption.")
        self.shutting_down = True
        self.publish_msg(self.message_type())
        
        if self.on_stop is not None:
            self.on_stop(self)
        
        time.sleep(1.0)

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Signal generator for corrupting messages')
    parser.add_argument(
        'output_topic', help='Output topic to publish corrupted messages')
    parser.add_argument(
        'message_type', help='Message type (e.g., nav_msgs/Odometry, sensor_msgs/Imu)')
    parser.add_argument(
        'field', help='Field to corrupt (e.g., orientation for Odometry)')
    parser.add_argument('signal_type', choices=[SIGNAL_TYPE_STEP, SIGNAL_TYPE_RAMP, SIGNAL_TYPE_OSCILLATING],
                        help='Type of signal to generate')
    parser.add_argument('magnitude', type=float,
                        help='Magnitude of the signal')
    parser.add_argument('--period', type=float, default=None,
                        help='Period of oscillating signal (optional)')
    parser.add_argument('--corruption_start_sec', type=float, default=2.0, help="Number of seconds to wait before starting to corrupt the message")
    args = parser.parse_args(myargv[1:])

    # Initialize ROS node
    rospy.init_node('signal_generator_node')

    # Wait for time to be non-zero
    while rospy.Time.now().to_sec() == 0:
        time.sleep(0.1)
        rospy.logwarn_throttle(1.0, "Initializing... for time to be non-zero.")
        pass

    now = rospy.Time.now()

    corruption_id = f"{now.secs}_{now.nsecs}_{args.field}_{args.signal_type}_{args.magnitude}"

    metadata_pub = rospy.Publisher('/metadata', Metadata, queue_size=10, latch=True)

    def publish_event(event_name, extra_metadata={}):
        """
        Publishes a metadata message with the given event name and extra metadata.
        """
        metadata = Metadata()
        metadata.header.stamp = rospy.Time.now()
        metadata.metadata = json.dumps({
            'metadata_type': event_name,
            'corruption_id': corruption_id,
            **extra_metadata
        })
        metadata_pub.publish(metadata)
    
    publish_event('corruption_init', {'args': vars(args)})
    
    # Create SignalGeneratorNode instance
    signal_generator_node = SignalGeneratorNode(
        args.output_topic,
        args.message_type,
        args.field,
        args.signal_type,
        args.magnitude,
        args.period,
        args.corruption_start_sec,
        on_start=lambda _node: publish_event('corruption_start'),
        on_stop=lambda _node: publish_event('corruption_stop'),
    )

    # Set up a timer to generate and publish signal at a fixed rate (e.g., 10 Hz)
    rospy.Timer(rospy.Duration(0.1),
                signal_generator_node.generate_and_publish_signal)

    rospy.on_shutdown(signal_generator_node.on_shutdown)

    # Keep node running
    rospy.spin()
