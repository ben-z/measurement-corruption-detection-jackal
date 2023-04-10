import sys
import transformations as tf

from collections import defaultdict
from pathlib import Path
from rosbags.highlevel import AnyReader

from typing import Dict, Optional, List, NamedTuple
from enum import Enum

DATA_PATH=Path('~/Downloads/masc-bags/').expanduser()
# BAG_NAME='2023-03-27-15-28-54_attacks-closed-loop-video-sync.bag'
BAG_NAME='2023-03-27-15-41-01_attacks-open-loop-video-sync.bag'
BAG_PATH=DATA_PATH / BAG_NAME

S_TO_NS = 1e9

# class CorruptionType(str, Enum):
#     ORIENTATION = 'ORIENTATION'
#     VELOCITY = 'velocity'
#     ANGULAR_VELOCITY = 'angular_velocity'

class Corruption(NamedTuple):
    begin_ns: int
    end_ns: Optional[int]

# Read the bag file
with AnyReader([BAG_PATH]) as reader:
    # Read the rosbag metadata to get start and end timestamps
    start_time_s = reader.start_time / S_TO_NS
    end_time_s = reader.end_time / S_TO_NS
    duration_s = end_time_s - start_time_s
    print(f"Start time (s): {start_time_s}, end time (s): {end_time_s}, duration (s): {duration_s}")

    corruption_connections = [c for c in reader.connections if c.topic.endswith('/corruption')]
    print(f"Found corruption topics: {set([c.topic for c in corruption_connections])}")

    corruptions: Dict[str, List[Corruption]] = {}
    print("Looking for corruption messages...")
    for connection, timestamp, rawdata in reader.messages(connections=corruption_connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        topic = connection.topic

        # print(f"Found corruption message on topic {topic} at timestamp {timestamp} ({timestamp / S_TO_NS - start_time_s:.2f} seconds into the rosbag)")
        if topic == '/global_localization/robot/odom/corruption':
            orientation = msg.pose.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _,_,yaw_corruption = tf.euler_from_quaternion(quaternion, axes='xyzs')
            print(f"[{timestamp} ({timestamp / S_TO_NS - start_time_s:.2f}s)] {topic} Corrupted heading: {yaw_corruption:.2f} quaternion: {quaternion}")
            if yaw_corruption == 0.0:
                # The corruption has ended
                if topic not in corruptions or len(corruptions[topic]) == 0 or corruptions[topic][-1].end_ns is not None:
                    print(f"WARNING: Found end corruption message on topic {topic} at timestamp {timestamp} ({timestamp / S_TO_NS - start_time_s:.2f} seconds into the rosbag) without a start message", file=sys.stderr)
                    continue
                corruptions[topic][-1] = corruptions[topic][-1]._replace(end_ns=timestamp)
            elif len(corruptions.get('topic', [])) > 0 and corruptions[topic][-1].end_ns is None:
                # The corruption is already in progress, this is a noop
                pass
            else:
                # The corruption has just started
                corruptions[topic] = corruptions.get(topic, []) + [Corruption(timestamp, None)]
        elif topic == '/jackal_velocity_controller/odom/corruption':
            velocity_corruption = msg.twist.twist.linear.x
            angular_velocity_corruption = msg.twist.twist.angular.z
            print(f"[{timestamp} ({timestamp / S_TO_NS - start_time_s:.2f}s)] {topic} Corrupted velocity: {velocity_corruption:.2f}, corrupted angular velocity: {angular_velocity_corruption:.2f}")
            if velocity_corruption == 0.0 and angular_velocity_corruption == 0.0:
                # The corruption has ended
                if topic not in corruptions or len(corruptions[topic]) == 0 or corruptions[topic][-1].end_ns is not None:
                    # The corruption has ended, but we haven't seen a start message yet
                    print(f"WARNING: Found end corruption message on topic {topic} at timestamp {timestamp} ({timestamp / S_TO_NS - start_time_s:.2f} seconds into the rosbag) without a start message", file=sys.stderr)
                    continue
                corruptions[topic][-1] = corruptions[topic][-1]._replace(end_ns=timestamp)
            elif len(corruptions.get('topic', [])) > 0 and corruptions[topic][-1].end_ns is None:
                # The corruption is already in progress, this is a noop
                pass
            else:
                # The corruption has just started
                corruptions[topic] = corruptions.get(topic, []) + [Corruption(timestamp, None)]
        elif topic == '/bbase/imu/data/corruption':
            angular_velocity_corruption = msg.angular_velocity.z
            print(f"[{timestamp} ({timestamp / S_TO_NS - start_time_s:.2f}s)] {topic} Corrupted angular velocity: {angular_velocity_corruption:.2f}")
            if angular_velocity_corruption == 0.0:
                # The corruption has ended
                if topic not in corruptions or len(corruptions[topic]) == 0 or corruptions[topic][-1].end_ns is not None:
                    # The corruption has ended, but we haven't seen a start message yet
                    print(f"WARNING: Found end corruption message on topic {topic} at timestamp {timestamp} ({timestamp / S_TO_NS - start_time_s:.2f} seconds into the rosbag) without a start message", file=sys.stderr)
                    continue
                corruptions[topic][-1] = corruptions[topic][-1]._replace(end_ns=timestamp)
            elif len(corruptions.get('topic', [])) > 0 and corruptions[topic][-1].end_ns is None:
                # The corruption is already in progress, this is a noop
                pass
            else:
                # The corruption has just started
                corruptions[topic] = corruptions.get(topic, []) + [Corruption(timestamp, None)]
        else:
            raise ValueError(f"Unknown corruption topic: {topic}")
