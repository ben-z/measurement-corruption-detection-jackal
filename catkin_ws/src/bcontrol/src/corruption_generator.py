#!/usr/bin/env python3

import rospy
import numpy as np
import argparse
from .signal_generator import corrupt_array, SignalType, SIGNAL_TYPE_STRS
from .numpy_message_converter import OdometryConverter, ImuConverter
from .ros_utils import load_message_type
import time
from copy import deepcopy
import sys
from bcontrol.msg import Metadata
import json
from typing import NamedTuple, List, Optional, Callable

class CorruptionGeneratorSpec(NamedTuple):
    signal_type: SignalType
    magnitude: float
    period: Optional[float]
    output_topic: str
    message_type: str
    field: str
    corruption_start_sec: float

# Define message converters for supported message types
MESSAGE_CONVERTERS = {
    'nav_msgs/Odometry': OdometryConverter,
    'sensor_msgs/Imu': ImuConverter,
}

CallbackType = Callable[['CorruptionGeneratorNode'], None]

class CorruptionGeneratorNode:
    def __init__(self, spec: CorruptionGeneratorSpec, on_start: Optional[CallbackType] = None, on_stop: Optional[CallbackType] = None):
        self.spec = spec
        self.called_start_callback = False
        self.on_start = on_start
        self.on_stop = on_stop
        self.shutting_down = False

    def init(self):
        spec = self.spec

        # Wait for time to be non-zero
        while rospy.Time.now().to_sec() == 0:
            time.sleep(0.1)
            rospy.logwarn_throttle(1.0, "Initializing... waiting for time to be non-zero.")
            pass

        now = rospy.Time.now()
        self.corruption_id = f"{now.secs}_{now.nsecs}_{spec.field}_{spec.signal_type}_{spec.magnitude}"

        message_type = spec.message_type
        field = spec.field
        output_topic = spec.output_topic
        corruption_start_sec = spec.corruption_start_sec

        # Determine the message converter based on the message type
        if message_type in MESSAGE_CONVERTERS:
            self.message_converter = MESSAGE_CONVERTERS[message_type]()
        else:
            raise ValueError(f"Unsupported message type '{message_type}'")

        # Get indices for the specified field
        self.indices = self.message_converter.get_indices(field)
        self.signal_specs = [{'signal_type': self.spec.signal_type,
                                  'magnitude': self.spec.magnitude, 'indices': self.indices, 'period': self.spec.period}]

        # Resolve message type and create publisher
        MessageType = load_message_type(message_type)
        self.corrupted_pub = rospy.Publisher(output_topic, MessageType, queue_size=10)
        
        # Create metadata publisher
        self.metadata_pub = rospy.Publisher('/metadata', Metadata, queue_size=10, latch=True)

        # Create message template
        self.message_type = MessageType

        self.init_time = rospy.get_time() + corruption_start_sec

        self.publish_event('corruption_init')

        # Set up a timer to generate and publish signal at a fixed rate (e.g., 10 Hz)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.generate_and_publish_signal)

        rospy.on_shutdown(self.shutdown)

    def generate_and_publish_signal(self, event):
        if self.shutting_down:
            return
        t = rospy.get_time() - self.init_time
        if t < 0:
            rospy.logwarn_throttle(1.0, f"Corruption starting in {-t:.2f} secs.")
            self.publish_msg(self.message_type())
            return
        np_data = self.message_converter.to_numpy(self.message_type())
        corrupted_data = corrupt_array(np_data, self.signal_specs, t)
        corrupted_msg = self.message_converter.from_numpy(
            corrupted_data, self.message_type())
        
        rospy.logwarn_once("Publishing corrupted message.")
        self.publish_msg(corrupted_msg)
        
        if not self.called_start_callback:
            self.publish_event('corruption_start')
            if self.on_start is not None:
                self.on_start(self)
            self.called_start_callback = True

    def publish_msg(self, msg):
        # If a header is present, set the timestamp
        if hasattr(msg, 'header'):
            msg.header.stamp = rospy.Time.now()

        self.corrupted_pub.publish(msg)

    def shutdown(self):
        if self.shutting_down:
            return
        self._shutdown()

    def _shutdown(self):
        # Publish the template message when shutting down
        rospy.logwarn("Shutting down, clearing corruption.")
        self.shutting_down = True
        self.publish_msg(self.message_type())
        
        self.publish_event('corruption_stop')
        if self.on_stop is not None:
            self.on_stop(self)
        
        time.sleep(1.0)
    
    def publish_event(self, event_name, extra_metadata={}):
        """
        Publishes a metadata message with the given event name and extra metadata.
        """
        metadata = Metadata()
        metadata.header.stamp = rospy.Time.now()
        metadata.metadata = json.dumps({
            'metadata_type': event_name,
            'corruption_id': self.corruption_id,
            'spec': self.spec._asdict(),
            **extra_metadata
        })
        self.metadata_pub.publish(metadata)

def create_parser(**kwargs):
    parser = argparse.ArgumentParser(
        description='Signal generator for corrupting messages', **kwargs)
    parser.add_argument(
        'output_topic', help='Output topic to publish corrupted messages')
    parser.add_argument(
        'message_type', help='Message type (e.g., nav_msgs/Odometry, sensor_msgs/Imu)')
    parser.add_argument(
        'field', help='Field to corrupt (e.g., orientation for Odometry)')
    parser.add_argument('signal_type', choices=SIGNAL_TYPE_STRS,
                        help='Type of signal to generate')
    parser.add_argument('magnitude', type=float,
                        help='Magnitude of the signal')
    parser.add_argument('--period', type=float, default=None,
                        help='Period of oscillating signal (optional)')
    parser.add_argument('--corruption_start_sec', type=float, default=2.0, help="Number of seconds to wait before starting to corrupt the message")

    return parser

def parse_cli_args(argv):
    # Parse command-line arguments
    parser = create_parser()
    args = parser.parse_args(argv)

    return create_spec(args)

def create_spec(args):
    return CorruptionGeneratorSpec(
        output_topic=args.output_topic,
        message_type=args.message_type,
        field=args.field,
        signal_type=args.signal_type,
        magnitude=args.magnitude,
        period=args.period,
        corruption_start_sec=args.corruption_start_sec,
    )

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    spec = parse_cli_args(myargv[1:])

    # Initialize ROS node
    rospy.init_node('signal_generator_node')

    # Create a CorruptionGeneratorNode instance
    node = CorruptionGeneratorNode(spec)
    node.init()

    # Keep node running
    rospy.spin()
