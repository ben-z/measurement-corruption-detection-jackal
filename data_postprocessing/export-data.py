# %%
import csv
import sys
import time
from collections import defaultdict
from enum import Enum
from pathlib import Path
from typing import Dict, List, NamedTuple, Optional

import numpy as np
import transformations as tf
from rosbags.highlevel import AnyReader
from tqdm import tqdm
from utils import (euler_from_quaternion, flatten_dict, ns_to_relative_s,
                   ns_to_s, read_messages_by_topics, s_to_ns, stamp_to_ns)

# %%
# Load data

BAG_PATH = Path(
    # "/Users/ben/Library/CloudStorage/GoogleDrive-ben@benzhang.dev/Shared drives/MASc Data/MASc Data/attacks-open-loop-video-sync_2023-03-27-15-41-01.bag"
    "/Users/ben/Library/CloudStorage/GoogleDrive-ben@benzhang.dev/Shared drives/MASc Data/MASc Data/attacks-closed-loop-video-sync_2023-03-27-15-28-54.bag"
)
S_TO_NS = 1e9

messages = read_messages_by_topics(
    BAG_PATH,
    [
        "/vicon/ben_jackal2/ben_jackal2/odom",
        "/global_localization/robot/odom",
        "/global_localization/robot/odom/vulnerable/ORIENTATION",
        "/global_localization/robot/odom/corruption",
        "/global_localization/robot/odom/vulnerable",
        "/bdetect/data",
        "/bdetect/sensor_validity",
        "/message_barrier/sensor_validity_final",
        "/bdetect/sensor_malfunction_max_magnitude",
    ],
)

# %%
# Pre-processing visualization

bdetect_debug_posearray = messages["/bdetect/data"][-1]
import matplotlib.pyplot as plt

# Extract the data from the PoseArray message
poses = bdetect_debug_posearray["msg"].poses
timestamps = np.array([ns_to_s(bdetect_debug_posearray["timestamp"])] * len(poses))
# # Create relative timestamps (0 to N-1) since these are historical poses
relative_timestamps = np.arange(len(poses))

# Extract x, y, and yaw from poses
x_values = np.array([pose.position.x for pose in poses])
y_values = np.array([pose.position.y for pose in poses])
yaw_values = np.array([euler_from_quaternion(pose.orientation)[2] for pose in poses])
# Unwrap yaw to avoid discontinuities
yaw_values = np.unwrap(yaw_values)

# Create a figure with 3 subplots
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Plot x position over time
axs[0].step(relative_timestamps, x_values, where='post', color='blue')
axs[0].set_ylabel('X Position (m)')
axs[0].set_title('X Position over Time')
axs[0].grid(True)

# Plot y position over time
axs[1].step(relative_timestamps, y_values, where='post', color='green')
axs[1].set_ylabel('Y Position (m)')
axs[1].set_title('Y Position over Time')
axs[1].grid(True)

# Plot yaw over time
axs[2].step(relative_timestamps, yaw_values, where='post', color='red')
axs[2].set_ylabel('Yaw (rad)')
axs[2].set_title('Yaw over Time')
axs[2].set_xlabel('Time Steps')
axs[2].grid(True)

plt.tight_layout()
plt.show()


# %%
# Construct data from messages

heading_corruption = np.unwrap([euler_from_quaternion(x["msg"].pose.pose.orientation)[2] for x in messages["/global_localization/robot/odom/corruption"]])
heading_corruption_start_ns = next(m["timestamp"] for c, m in zip(heading_corruption, messages["/global_localization/robot/odom/corruption"]) if c > 0)

data = {
    "vicon_ground_truth_odom": {
        "t": np.array([x["timestamp"] for x in messages["/vicon/ben_jackal2/ben_jackal2/odom"]]),
        "x": np.array([x["msg"].pose.pose.position.x for x in messages["/vicon/ben_jackal2/ben_jackal2/odom"]]),
        "y": np.array([x["msg"].pose.pose.position.y for x in messages["/vicon/ben_jackal2/ben_jackal2/odom"]]),
        "heading": np.unwrap([euler_from_quaternion(x["msg"].pose.pose.orientation)[2] for x in messages["/vicon/ben_jackal2/ben_jackal2/odom"]]),
    },
    "vicon_odom": {
        "t": np.array([x["timestamp"] for x in messages["/global_localization/robot/odom"]]),
        "x": np.array([x["msg"].pose.pose.position.x for x in messages["/global_localization/robot/odom"]]),
        "y": np.array([x["msg"].pose.pose.position.y for x in messages["/global_localization/robot/odom"]]),
        "heading": np.unwrap([euler_from_quaternion(x["msg"].pose.pose.orientation)[2] for x in messages["/global_localization/robot/odom"]]),
    },
    "vicon_odom_corruption": {
        "t": np.array([x["timestamp"] for x in messages["/global_localization/robot/odom/corruption"]]),
        "heading": heading_corruption,
    },
    "vicon_odom_vulnerable_orientation": {
        "t": np.array([x["timestamp"] for x in messages["/global_localization/robot/odom/vulnerable/ORIENTATION"]]),
        "heading": np.array([euler_from_quaternion(x["msg"].pose.pose.orientation)[2] for x in messages["/global_localization/robot/odom/vulnerable/ORIENTATION"]]),
    },
    "sensor_validity": {
        "t": np.array([x["timestamp"] for x in messages["/bdetect/sensor_validity"]]),
        "validity": np.array([",".join(str(n) for n in x["msg"].data.tolist()) for x in messages["/bdetect/sensor_validity"]]),
    },
    "sensor_validity_final": {
        "t": np.array([x["timestamp"] for x in messages["/message_barrier/sensor_validity_final"]]),
        "validity": np.array([",".join(str(n) for n in x["msg"].data.tolist()) for x in messages["/message_barrier/sensor_validity_final"]]),
    },
    "sensor_malfunction_max_magnitude": {
        "t": np.array([x["timestamp"] for x in messages["/bdetect/sensor_malfunction_max_magnitude"]]),
        "magnitude": np.array([",".join(str(n) for n in x["msg"].data.tolist()) for x in messages["/bdetect/sensor_malfunction_max_magnitude"]]),
    },
}

# Save data
np.savez_compressed(
    BAG_PATH.stem + '.npz',
    **flatten_dict(data)
)

# %%
# Preliminary analysis

import matplotlib.pyplot as plt

visualization_start_ns = heading_corruption_start_ns - s_to_ns(10)
visualization_end_ns = heading_corruption_start_ns + s_to_ns(10)

def filter_data_for_visualization(data, start_ns, end_ns):
    ret = {}
    for k, v in data.items():
        mask = (start_ns < v["t"]) & (v["t"] < end_ns)
        ret[k] = {k2: v2[mask] for k2, v2 in v.items()}
    return ret

def normalize_angle(angle):
    """
    Normalize angle to be between -pi and pi

    Derived from: https://stackoverflow.com/a/2321125
    """
    return np.unwrap(np.arctan2(np.sin(angle), np.cos(angle)))

visualization_data = filter_data_for_visualization(data, visualization_start_ns, visualization_end_ns)

# plot BEV
plt.figure()
plt.plot(visualization_data["vicon_ground_truth_odom"]["x"], visualization_data["vicon_ground_truth_odom"]["y"], label="VICON GROUND TRUTH")
plt.plot(visualization_data["vicon_odom"]["x"], visualization_data["vicon_odom"]["y"], label="VICON ODOM")
plt.legend()
plt.axis('equal')
plt.show()

# plot heading
# Create a figure with two subplots (top and bottom)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top subplot for heading data
ax1.plot(ns_to_relative_s(visualization_data["vicon_ground_truth_odom"]["t"], heading_corruption_start_ns), normalize_angle(visualization_data["vicon_ground_truth_odom"]["heading"]), label="VICON GROUND TRUTH")
ax1.plot(ns_to_relative_s(visualization_data["vicon_odom"]["t"], heading_corruption_start_ns), normalize_angle(visualization_data["vicon_odom"]["heading"]), label="VICON ODOM")
ax1.plot(ns_to_relative_s(visualization_data["vicon_odom_vulnerable_orientation"]["t"], heading_corruption_start_ns), normalize_angle(visualization_data["vicon_odom_vulnerable_orientation"]["heading"]), label="VICON ODOM VULNERABLE ORIENTATION")
ax1.plot(ns_to_relative_s(visualization_data["vicon_odom_corruption"]["t"], heading_corruption_start_ns), normalize_angle(visualization_data["vicon_odom_corruption"]["heading"]), '--', label="VICON ODOM CORRUPTION")
ax1.legend()
ax1.set_title("Heading Data")
ax1.grid(True)
# Bottom subplot for sensor validity data
ax2.plot(ns_to_relative_s(visualization_data["sensor_validity"]["t"], heading_corruption_start_ns), visualization_data["sensor_validity"]["validity"], label="SENSOR VALIDITY")
ax2.plot(ns_to_relative_s(visualization_data["sensor_validity_final"]["t"], heading_corruption_start_ns), visualization_data["sensor_validity_final"]["validity"], label="SENSOR VALIDITY FINAL")
ax2.legend()
ax2.set_title("Sensor Validity Data")
ax2.grid(True)
plt.tight_layout()
plt.show()

# plt.figure()
# plt.plot(ns_to_relative_s(visualization_data["sensor_malfunction_max_magnitude"]["t"], heading_corruption_start_ns), visualization_data["sensor_malfunction_max_magnitude"]["magnitude"], label="SENSOR MALFUNCTION MAX MAGNITUDE")
# plt.legend()
# plt.show()


# Define output CSV file name
# combined_csv = BAG_PATH.stem + '.csv'

# # Create a dictionary to store data for each timestamp
# # data_dict: Dict[int, Dict[str, str | float | None]] = defaultdict(lambda: {
# #     'x': None,
# #     'y': None,
# #     'ground_truth_heading': None,
# #     'corrupted_heading': None,
# #     'heading_corruption': None,
# #     'bdetect_sensor_validity': None,
# #     'message_barrier_sensor_validity_final': None,
# #     'sensor_malfunction_max_magnitude': None
# # })


# # class CorruptionType(str, Enum):
# #     ORIENTATION = 'ORIENTATION'
# #     VELOCITY = 'velocity'
# #     ANGULAR_VELOCITY = 'angular_velocity'

# class Corruption(NamedTuple):
#     begin_ns: int
#     end_ns: Optional[int]

# # Read the bag file
# with AnyReader([BAG_PATH]) as reader:
#     # Read the rosbag metadata to get start and end timestamps
#     start_time_s = reader.start_time / S_TO_NS
#     end_time_s = reader.end_time / S_TO_NS
#     duration_s = end_time_s - start_time_s
#     print(f"Start time (s): {start_time_s}, end time (s): {end_time_s}, duration (s): {duration_s}")

#     with tqdm(total=duration_s, desc="Processing rosbag", unit="s") as pbar:
#         for connection, timestamp, rawdata in reader.messages():
#             # Update the progress bar based on the current timestamp
#             pbar.update(timestamp / S_TO_NS - pbar.n - start_time_s)

#             msg = reader.deserialize(rawdata, connection.msgtype)
#             topic = connection.topic

#             # Extract data for bird's eye view
#             if topic == '/vicon/ben_jackal2/ben_jackal2/odom':
#                 data_dict[]
#                 data_dict[timestamp]['x'] = msg.pose.pose.position.x
#                 data_dict[timestamp]['y'] = msg.pose.pose.position.y

#             # Extract data for ground truth and corrupted heading
#             if topic == '/global_localization/robot/odom':
#                 orientation = msg.pose.pose.orientation
#                 quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
#                 _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
#                 data_dict[timestamp]['ground_truth_heading'] = yaw
#             if topic == '/global_localization/robot/odom/corruption':
#                 orientation = msg.pose.pose.orientation
#                 quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
#                 _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
#                 data_dict[timestamp]['heading_corruption'] = yaw
#             if topic == '/global_localization/robot/odom/vulnerable/ORIENTATION':
#                 orientation = msg.pose.pose.orientation
#                 quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
#                 _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
#                 data_dict[timestamp]['corrupted_heading'] = yaw

#             # Extract data for sensor validity
#             if topic == '/bdetect/sensor_validity':
#                 data_dict[timestamp]['bdetect_sensor_validity'] = ",".join(str(n) for n in msg.data.tolist())
#             if topic == '/message_barrier/sensor_validity_final':
#                 data_dict[timestamp]['message_barrier_sensor_validity_final'] = ",".join(str(n) for n in msg.data.tolist())

#             # Extract data for sensor malfunction max magnitude
#             if topic == '/bdetect/sensor_malfunction_max_magnitude':
#                 data_dict[timestamp]['sensor_malfunction_max_magnitude'] = ",".join(str(n) for n in msg.data.tolist())

#             # if topic == '/global_localization/robot/odom/corruption'

# # Write data to the CSV file
# with open(combined_csv, 'w', newline='') as csvfile:
#     fieldnames = [
#         'timestamp',
#         'x',
#         'y',
#         'ground_truth_heading',
#         'corrupted_heading',
#         'heading_corruption',
#         'bdetect_sensor_validity',
#         'message_barrier_sensor_validity_final',
#         'sensor_malfunction_max_magnitude'
#     ]
#     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#     writer.writeheader()

#     # Write data sorted by timestamp
#     for timestamp, data in sorted(data_dict.items()):
#         row = {'timestamp': timestamp, **data}
#         writer.writerow(row)

# print(f'Data saved to {combined_csv}')
