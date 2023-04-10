#!/usr/bin/env python

from rosbags.rosbag1 import Writer
from rosbags.serde import deserialize_cdr, ros1_to_cdr, cdr_to_ros1, serialize_cdr
from rosbags.highlevel import AnyReader as Reader
from pathlib import Path
from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
import transformations as tf
from tqdm import tqdm
import argparse

S_TO_NS = 1e9

def orientation_to_rpy_vec3(orientation) -> Vector3:
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
    return Vector3(x=roll, y=pitch, z=yaw)

def main(rosbag_path: Path, output_path: Path):
    # delete augmented bag if it exists
    output_path.unlink(missing_ok=True)

    # create reader instance
    with Reader([rosbag_path]) as reader, Writer(output_path) as writer:
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str, help='Path to the rosbag to augment')
    parser.add_argument('--output_path', type=str, required=False, help='Path to the augmented rosbag (default: <rosbag_path>.augmented.bag)')
    args = parser.parse_args()

    rosbag_path = Path(args.rosbag_path)
    if not rosbag_path.exists():
        raise FileNotFoundError(f"Rosbag path {rosbag_path} does not exist")
    
    if args.output_path:
        output_path = Path(args.output_path)
    else:
        output_path = rosbag_path.with_suffix('.augmented.bag')

    main(rosbag_path, output_path)
