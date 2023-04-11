#!/usr/bin/env python

# Register custom message types
from pathlib import Path
from rosbags.typesys import get_types_from_msg, register_types

SCRIPT_DIR = Path(__file__).parent

BCONTROL_MSG_DIR = SCRIPT_DIR / '../catkin_ws/src/bcontrol/msg/'
assert BCONTROL_MSG_DIR.exists(), f"bcontrol msg directory {BCONTROL_MSG_DIR} does not exist"

def infer_msg_name(path: Path) -> str:
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

add_types = {}
for msgpath in BCONTROL_MSG_DIR.glob('*.msg'):
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, infer_msg_name(msgpath)))
register_types(add_types)

from rosbags.rosbag1 import Writer
from rosbags.interfaces import ConnectionExtRosbag1
from rosbags.serde import deserialize_cdr, ros1_to_cdr, cdr_to_ros1, serialize_cdr
from rosbags.highlevel import AnyReader as Reader
from pathlib import Path
from rosbags.typesys.types import \
    geometry_msgs__msg__Vector3 as Vector3, \
    geometry_msgs__msg__PoseStamped as PoseStamped, \
    bcontrol__msg__Path as PathMsg, \
    nav_msgs__msg__Path as NavPathMsg
import transformations as tf
from tqdm import tqdm
import argparse
import os
import sys

is_interactive = os.isatty(sys.stdout.fileno()) and os.environ.get('SHOW_PROGRESS', 'true') == 'true'

S_TO_NS = 1e9

def orientation_to_rpy_vec3(orientation) -> Vector3:
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
    return Vector3(x=roll, y=pitch, z=yaw)

def main(rosbag_path: Path, output_path: Path):
    print(f"Augmenting {rosbag_path}... Output path: {output_path}")

    # delete augmented bag if it exists
    output_path.unlink(missing_ok=True)

    with Reader([rosbag_path]) as reader, Writer(output_path) as writer:
        start_time = reader.start_time
        start_time_s = start_time / S_TO_NS
        end_time_s = reader.end_time / S_TO_NS
        duration_s = end_time_s - start_time_s
        print(f"Start time (s): {start_time_s:.4f}, end time (s): {end_time_s:.4f}, duration (s): {duration_s:.4f}")

        # copy connections from reader to writer
        writer_connections = {}
        for connection in reader.connections:
            assert isinstance(connection.ext, ConnectionExtRosbag1), "Only rosbag1 connections are supported"
            wconn = writer.add_connection(
                topic=connection.topic,
                msgtype=connection.msgtype,
                msgdef=connection.msgdef,
                md5sum=connection.md5sum,
                callerid=connection.ext.callerid,
                latching=connection.ext.latching,
            )
            writer_connections[(connection.topic, connection.msgtype)] = wconn

        all_topics = {x.topic for x in reader.connections}

        # Quaternion to RPY
        odometry_connections = [x for x in reader.connections if x.msgtype == 'nav_msgs/msg/Odometry']
        quaternion_to_rpy_connection_mapping = {}
        for connection in odometry_connections:
            topic = connection.topic + '/orientation_rpy'
            if topic in all_topics:
                print(f"Topic {topic} already exists, skipping this transformation")
                continue
            msgtype = Vector3.__msgtype__
            new_connection = writer.add_connection(topic, msgtype)
            quaternion_to_rpy_connection_mapping[connection.topic] = (new_connection, msgtype)

        # Path to NavPath
        path_connections = [x for x in reader.connections if x.msgtype == 'bcontrol/msg/Path']
        path_to_navpath_connection_mapping = {}
        for connection in path_connections:
            topic = connection.topic + '/nav_path'
            if topic in all_topics:
                print(f"Topic {topic} already exists, skipping this transformation")
                continue
            msgtype = NavPathMsg.__msgtype__
            new_connection = writer.add_connection(topic, msgtype)
            path_to_navpath_connection_mapping[connection.topic] = (new_connection, msgtype)

        with tqdm(total=round(duration_s,4), desc="Processing rosbag", unit="s", disable=not is_interactive) as pbar:
            last_progress_update_ns_from_start = -1

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                ns_from_start = timestamp - start_time

                pbar.update(round(ns_from_start / S_TO_NS - pbar.n, 4))

                if not is_interactive and ns_from_start % (10 * S_TO_NS) == 0 and ns_from_start > last_progress_update_ns_from_start:
                    print(f"Progress update [{rosbag_path.name}]: processing {ns_from_start / S_TO_NS:.4f}/{duration_s:.4f} seconds")
                    last_progress_update_ns_from_start = ns_from_start

                if quaternion_to_rpy_connection_mapping.get(connection.topic):
                    out_connection, out_msgtype = quaternion_to_rpy_connection_mapping[connection.topic]
                    msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    vec3 = orientation_to_rpy_vec3(msg.pose.pose.orientation)
                    writer.write(out_connection, timestamp, cdr_to_ros1(serialize_cdr(vec3, out_msgtype), out_msgtype))
                elif path_to_navpath_connection_mapping.get(connection.topic):
                    out_connection, out_msgtype = path_to_navpath_connection_mapping[connection.topic]
                    msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    navpath = NavPathMsg(
                        header=msg.header,
                        poses=[PoseStamped(header=msg.header, pose=pose) for pose in msg.poses]
                    )
                    writer.write(out_connection, timestamp, cdr_to_ros1(serialize_cdr(navpath, out_msgtype), out_msgtype))
                else:
                    pass

                wconn = writer_connections[(connection.topic, connection.msgtype)]
                writer.write(wconn, timestamp, rawdata)

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
