import csv
import sys
import transformations as tf
from tqdm import tqdm

from collections import defaultdict
from pathlib import Path
from rosbags.highlevel import AnyReader

from typing import Dict, Optional, List, NamedTuple
from enum import Enum

# DATA_PATH=Path('~/Downloads/masc-bags/').expanduser()
# BAG_NAME='2023-03-27-15-41-01_attacks-open-loop-video-sync.bag'
# # BAG_NAME='2023-03-27-15-28-54_attacks-closed-loop-video-sync.bag'
# BAG_PATH=DATA_PATH / BAG_NAME
BAG_PATH = Path(
    # "/Users/ben/Library/CloudStorage/GoogleDrive-ben@benzhang.dev/Shared drives/MASc Data/MASc Data/attacks-open-loop-video-sync_2023-03-27-15-41-01.bag"
    "/Users/ben/Library/CloudStorage/GoogleDrive-ben@benzhang.dev/Shared drives/MASc Data/MASc Data/attacks-closed-loop-video-sync_2023-03-27-15-28-54.bag"
)

S_TO_NS = 1e9

# Define output CSV file name
combined_csv = BAG_PATH.stem + '.csv'

# Create a dictionary to store data for each timestamp
data_dict: Dict[int, Dict[str, str | float | None]] = defaultdict(lambda: {
    'x': None,
    'y': None,
    'ground_truth_heading': None,
    'corrupted_heading': None,
    'heading_corruption': None,
    'bdetect_sensor_validity': None,
    'message_barrier_sensor_validity_final': None,
    'sensor_malfunction_max_magnitude': None
})

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
    
    with tqdm(total=duration_s, desc="Processing rosbag", unit="s") as pbar:
        for connection, timestamp, rawdata in reader.messages():
            # Update the progress bar based on the current timestamp
            pbar.update(timestamp / S_TO_NS - pbar.n - start_time_s)

            msg = reader.deserialize(rawdata, connection.msgtype)
            topic = connection.topic

            # Extract data for bird's eye view
            if topic == '/vicon/ben_jackal2/ben_jackal2/odom':
                data_dict[timestamp]['x'] = msg.pose.pose.position.x
                data_dict[timestamp]['y'] = msg.pose.pose.position.y
            
            # Extract data for ground truth and corrupted heading
            if topic == '/global_localization/robot/odom':
                orientation = msg.pose.pose.orientation
                quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
                _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
                data_dict[timestamp]['ground_truth_heading'] = yaw
            if topic == '/global_localization/robot/odom/corruption':
                orientation = msg.pose.pose.orientation
                quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
                _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
                data_dict[timestamp]['heading_corruption'] = yaw
            if topic == '/global_localization/robot/odom/vulnerable/ORIENTATION':
                orientation = msg.pose.pose.orientation
                quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
                _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
                data_dict[timestamp]['corrupted_heading'] = yaw
            
            # Extract data for sensor validity
            if topic == '/bdetect/sensor_validity':
                data_dict[timestamp]['bdetect_sensor_validity'] = ",".join(str(n) for n in msg.data.tolist())
            if topic == '/message_barrier/sensor_validity_final':
                data_dict[timestamp]['message_barrier_sensor_validity_final'] = ",".join(str(n) for n in msg.data.tolist())
            
            # Extract data for sensor malfunction max magnitude
            if topic == '/bdetect/sensor_malfunction_max_magnitude':
                data_dict[timestamp]['sensor_malfunction_max_magnitude'] = ",".join(str(n) for n in msg.data.tolist())

            # if topic == '/global_localization/robot/odom/corruption'

# Write data to the CSV file
with open(combined_csv, 'w', newline='') as csvfile:
    fieldnames = [
        'timestamp',
        'x',
        'y',
        'ground_truth_heading',
        'corrupted_heading',
        'heading_corruption',
        'bdetect_sensor_validity',
        'message_barrier_sensor_validity_final',
        'sensor_malfunction_max_magnitude'
    ]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    
    # Write data sorted by timestamp
    for timestamp, data in sorted(data_dict.items()):
        row = {'timestamp': timestamp, **data}
        writer.writerow(row)

print(f'Data saved to {combined_csv}')
