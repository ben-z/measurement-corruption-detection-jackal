#!/usr/bin/env python

from rosbags.rosbag1 import Writer
from rosbags.serde import deserialize_cdr, ros1_to_cdr, cdr_to_ros1, serialize_cdr
from rosbags.highlevel import AnyReader as Reader
from pathlib import Path
from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
import transformations as tf
from tqdm import tqdm

S_TO_NS = 1e9

EXPERIMENTS_PATH = Path('./experiments').expanduser()
ROSBAG_PATH = EXPERIMENTS_PATH / '2023-04-10T17:54:09+00:00_myexp/2023-04-10T17:54:09+00:00-myexp.bag'
AUGMENTED_ROSBAG_PATH = ROSBAG_PATH.with_suffix('.augmented.bag')

# delete augmented bag if it exists
AUGMENTED_ROSBAG_PATH.unlink(missing_ok=True)

def orientation_to_rpy_vec3(orientation) -> Vector3:
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
    return Vector3(x=roll, y=pitch, z=yaw)

# create reader instance
with Reader([ROSBAG_PATH]) as reader, Writer(AUGMENTED_ROSBAG_PATH) as writer:
    start_time_s = reader.start_time / S_TO_NS
    end_time_s = reader.end_time / S_TO_NS
    duration_s = end_time_s - start_time_s
    print(f"Start time (s): {start_time_s}, end time (s): {end_time_s}, duration (s): {duration_s}")

    # copy connections from reader to writer
    writer.connections = reader.connections

    odometry_connections = [x for x in reader.connections if x.msgtype == 'nav_msgs/msg/Odometry']
    quaternion_to_rpy_connection_mapping = {}

    for connection in odometry_connections:
        topic = connection.topic + '/orientation_rpy'
        msgtype = Vector3.__msgtype__
        new_connection = writer.add_connection(topic, msgtype)
        quaternion_to_rpy_connection_mapping[connection.topic] = (new_connection, msgtype)

    with tqdm(total=round(duration_s,4), desc="Processing rosbag", unit="s") as pbar:
        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            pbar.update(round(timestamp / S_TO_NS - pbar.n - start_time_s, 4))

            if quaternion_to_rpy_connection_mapping.get(connection.topic):
                out_connection, out_msgtype = quaternion_to_rpy_connection_mapping[connection.topic]
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                vec3 = orientation_to_rpy_vec3(msg.pose.pose.orientation)
                writer.write(out_connection, timestamp, cdr_to_ros1(serialize_cdr(vec3, out_msgtype), out_msgtype))
            else:
                pass

            writer.write(connection, timestamp, rawdata)
