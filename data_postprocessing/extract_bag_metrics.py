#!/usr/bin/env python

# This script extracts metrics from bag files

import sys

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

# Add catkin_ws/src to PYTHONPATH
catkin_ws_src_path = SCRIPT_DIR.parent / 'catkin_ws/src'
if str(catkin_ws_src_path) not in sys.path:
    sys.path.append(str(catkin_ws_src_path))

from rosbags.rosbag1 import Writer
from rosbags.interfaces import ConnectionExtRosbag1
from rosbags.serde import deserialize_cdr, ros1_to_cdr, cdr_to_ros1, serialize_cdr
from rosbags.highlevel import AnyReader as Reader
from pathlib import Path
from rosbags.typesys.types import \
    builtin_interfaces__msg__Time as ROSTime, \
    geometry_msgs__msg__Vector3 as Vector3, \
    geometry_msgs__msg__PoseStamped as PoseStamped, \
    bcontrol__msg__Path as PathMsg, \
    bcontrol__msg__Float64Stamped as Float64Stamped, \
    bcontrol__msg__Metadata as Metadata, \
    nav_msgs__msg__Path as NavPathMsg
import transformations as tf
from tqdm import tqdm
import argparse
import os
import yaml
import json
import numpy as np
from typing import TypedDict, Optional, Dict, List
from bcontrol.src.detector_types import ModelConfig
from bcontrol.src.type_utils import typeguard

is_interactive = os.isatty(sys.stdout.fileno()) and os.environ.get('SHOW_PROGRESS', 'true') == 'true'

S_TO_NS: int = 1_000_000_000

class Metrics(TypedDict):
    corruption_init_ns: Optional[int]
    corruption_start_ns: Optional[int]
    corruption_stop_ns: Optional[int]
    first_sensor_timestamp_after_corruption: Optional[int]
    max_lateral_position_during_corruption: Optional[float]
    detection_result: Optional[str]
    detection_ns: Optional[int]
    misattributed_indices: Optional[List[int]]
    corruption_topic_base: Optional[str]
    corruption_idx: Optional[int]
    errors: List[str]

def ros_time_to_ns(ros_time: ROSTime) -> int:
    return ros_time.sec * S_TO_NS + ros_time.nanosec

# FIXME: this is a hack to map the corruption fields (from numpy_message_converter)
# to the state names used in the bdetect config.
# We should design something so that they know about each other.
corruption_field_to_state = {
    'linear_vel_x': 'VELOCITY',
    'orientation': 'ORIENTATION',
    'angular_vel_z': 'ANGULAR_VELOCITY',
}

def main(rosbag_path: Path, bdetect_config: ModelConfig, output_path: Path, overwrite: bool = False):
    print(f"Extracting metrics from {rosbag_path}... Output path: {output_path}")

    sensor_validity_config = []
    for sensor in bdetect_config['sensors']:
        for state in sensor['measured_states']:
            sensor_validity_config.append({
                'topic': sensor['topic'],
                'state': state,
            })

    # delete existing output file if it exists
    if output_path.exists():
        if overwrite:
            print(f"Deleting existing output file {output_path}")
            output_path.unlink()
        else:
            raise FileExistsError(f"Output file {output_path} already exists. Use --overwrite to overwrite it.")

    with Reader([rosbag_path]) as reader:
        start_time = reader.start_time
        start_time_s = start_time / S_TO_NS
        end_time_s = reader.end_time / S_TO_NS
        duration_s = end_time_s - start_time_s
        print(f"Start time (s): {start_time_s:.4f}, end time (s): {end_time_s:.4f}, duration (s): {duration_s:.4f}")

        all_topics = {x.topic for x in reader.connections}

        metrics: Metrics = {
            'corruption_init_ns': None,
            'corruption_start_ns': None,
            'corruption_stop_ns': None,
            'first_sensor_timestamp_after_corruption': None,
            'max_lateral_position_during_corruption': None,
            'detection_result': None, # detected, not detected, misattributed
            'detection_ns': None,
            'misattributed_indices': None,
            'corruption_topic_base': None,
            'corruption_idx': None,
            'errors': [],
        }

        with tqdm(total=round(duration_s,4), desc="Processing rosbag", unit="s", disable=not is_interactive) as pbar:
            last_progress_update_ns_from_start = -1

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                ns_from_start = timestamp - start_time

                pbar.update(round(ns_from_start / S_TO_NS - pbar.n, 4))

                if not is_interactive and ns_from_start % (10 * S_TO_NS) == 0 and ns_from_start > last_progress_update_ns_from_start:
                    print(f"Progress update [{rosbag_path.name}]: processing {ns_from_start / S_TO_NS:.4f}/{duration_s:.4f} seconds")
                    last_progress_update_ns_from_start = ns_from_start

                if connection.topic == '/metadata':
                    metadata_msg= deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    metadata_stamp = ros_time_to_ns(metadata_msg.header.stamp)
                    metadata = json.loads(metadata_msg.metadata)
                    if metadata['metadata_type'] == 'corruption_init':
                        assert metrics['corruption_init_ns'] is None, "This bag has multiple corruption init messages. This is not supported."
                        metrics['corruption_init_ns'] = metadata_stamp
                        assert metadata['spec']['output_topic'].endswith('/corruption')
                        metrics['corruption_topic_base'] = metadata['spec']['output_topic'][:-len('/corruption')]
                        corruption_field = metadata['spec']['field']
                        try:
                            metrics['corruption_idx'] = sensor_validity_config.index({
                                'topic': metrics['corruption_topic_base'] + '/vulnerable',
                                'state': corruption_field_to_state[corruption_field],
                            })
                        except ValueError:
                            raise ValueError(f"Could not find sensor validity config for {metrics['corruption_topic_base']}/vulnerable, {corruption_field_to_state[corruption_field]}")
                    elif metadata['metadata_type'] == 'corruption_start':
                        assert metrics['corruption_start_ns'] is None, "This bag has multiple corruption start messages. This is not supported."
                        metrics['corruption_start_ns'] = metadata_stamp
                        # TODO: extract the field, map it to the position in the sensor validity array
                    elif metadata['metadata_type'] == 'corruption_stop':
                        assert metrics['corruption_stop_ns'] is None, "This bag has multiple corruption end messages. This is not supported."
                        metrics['corruption_stop_ns'] = metadata_stamp

                if connection.topic == '/bcontrol/lateral_position' and metrics['corruption_start_ns'] is not None and metrics['corruption_stop_ns'] is None:
                    lateral_position = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype).data
                    if metrics['max_lateral_position_during_corruption'] is None or lateral_position > metrics['max_lateral_position_during_corruption']:
                        metrics['max_lateral_position_during_corruption'] = lateral_position
                
                if metrics['corruption_topic_base'] \
                    and connection.topic == metrics['corruption_topic_base'] + '/vulnerable' \
                    and metrics['first_sensor_timestamp_after_corruption'] is None \
                :
                    corrupted_msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    if hasattr(corrupted_msg, 'header'):
                        metrics['first_sensor_timestamp_after_corruption'] = ros_time_to_ns(corrupted_msg.header.stamp)
                    else:
                        metrics['first_sensor_timestamp_after_corruption'] = timestamp
                
                if metrics['corruption_start_ns'] and metrics['detection_ns'] is None and connection.topic == '/message_barrier/sensor_validity_final':
                    sensor_validity_msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    sensor_validity = np.array(sensor_validity_msg.data, dtype=bool)
                    invalid_sensors = np.where(sensor_validity == 0)[0]
                    if len(invalid_sensors) == 0:
                        # no corruption detected,
                        pass
                    elif len(invalid_sensors) > 1 or invalid_sensors[0] != metrics['corruption_idx']:
                        # corruption detected, but not at the right sensor
                        metrics['misattributed_indices'] = invalid_sensors.tolist()
                        metrics['detection_ns'] = timestamp
                        metrics['detection_result'] = 'misattributed'
                    else:
                        # corruption detected at the right sensor
                        assert len(invalid_sensors) == 1 and invalid_sensors[0] == metrics['corruption_idx'], f"Logic error: {invalid_sensors=}, {metrics['corruption_idx']=}"
                        metrics['detection_ns'] = timestamp
                        metrics['detection_result'] = 'detected'

        try:
            if metrics['corruption_init_ns'] is not None \
                or metrics['corruption_start_ns'] is not None \
                or metrics['corruption_stop_ns'] is not None \
            :
                assert metrics['corruption_init_ns'] is not None and metrics['corruption_start_ns'] is not None and metrics['corruption_stop_ns'] is not None, "This bag has corruption metadata but not all of the corruption metadata. This is not supported."
                assert metrics['corruption_start_ns'] > metrics['corruption_init_ns'], "This bag has an corruption start time before the corruption init time. This is not supported."
                assert metrics['corruption_stop_ns'] > metrics['corruption_start_ns'], "This bag has an corruption end time before the corruption start time. This is not supported."
        except AssertionError as e:
            print(f"Error validating the metrics for {rosbag_path}: {e}")
            metrics['errors'].append(str(e))

        with open(output_path, 'w') as f:
            yaml.safe_dump(metrics, f)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str, help='Path to the rosbag to augment')
    parser.add_argument('--bdetect_config_path', type=str, help='Path to the bdetect config file')
    parser.add_argument('--overwrite', action='store_true', help='Overwrite the augmented rosbag if it exists')
    args = parser.parse_args()

    rosbag_path = Path(args.rosbag_path)
    if not rosbag_path.exists():
        raise FileNotFoundError(f"Rosbag path {rosbag_path} does not exist")
    
    if args.bdetect_config_path is None:
        bdetect_config_path = SCRIPT_DIR.parent / "catkin_ws/src/bcontrol/config/bdetect.yaml"
    else:
        bdetect_config_path = Path(args.bdetect_config_path)
    
    if not bdetect_config_path.exists():
        raise FileNotFoundError(f"bdetect config path {bdetect_config_path} does not exist")
    
    with open(bdetect_config_path, 'r') as f:
        raw_bdetect_config = yaml.safe_load(f)

    bdetect_config: ModelConfig = typeguard.check_type(raw_bdetect_config.get('bdetect'), ModelConfig)
    
    output_path = rosbag_path.parent / 'metrics.yaml'

    main(rosbag_path, bdetect_config, output_path, overwrite=args.overwrite)
