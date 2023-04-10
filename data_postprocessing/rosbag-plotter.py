#%%
import matplotlib.pyplot as plt
import numpy as np
import transformations as tf

from pathlib import Path
from rosbags.highlevel import AnyReader

DATA_PATH=Path('~/Downloads/masc-bags/').expanduser()
BAG_PATH=DATA_PATH / '2023-03-27-15-28-54_attacks-closed-loop-video-sync.bag'

#%%
# Lists to store data for plotting
timestamps_heading = []
ground_truth_heading = []
corrupted_heading = []
x_positions = []
y_positions = []
timestamps_sensor_validity = []
bdetect_sensor_validity = []
message_barrier_sensor_validity_final = []
timestamps_sensor_malfunction = []
sensor_malfunction_max_magnitude = []

# Read the bag file
with AnyReader([BAG_PATH]) as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = reader.deserialize(rawdata, connection.msgtype)
        topic = connection.topic
        
        # Extract data for bird's eye view plot
        if topic == '/vicon/ben_jackal2/ben_jackal2/odom':
            x_positions.append(msg.pose.pose.position.x)
            y_positions.append(msg.pose.pose.position.y)
        
        # Extract data for ground truth and corrupted heading plot
        if topic == '/global_localization/robot/odom':
            orientation = msg.pose.pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
            timestamps_heading.append(timestamp)
            ground_truth_heading.append(yaw)
        elif topic == '/global_localization/robot/odom/vulnerable/ORIENTATION':
            orientation = msg.pose.pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            _, _, yaw = tf.euler_from_quaternion(quaternion, axes='xyzs')
            corrupted_heading.append(yaw)
        
        # Extract data for sensor validity plot
        if topic == '/bdetect/sensor_validity':
            timestamps_sensor_validity.append(timestamp)
            bdetect_sensor_validity.append(msg.data[0])  # Assuming validity of the first sensor
        elif topic == '/message_barrier/sensor_validity_final':
            message_barrier_sensor_validity_final.append(msg.data[0])  # Assuming validity of the first sensor
        
        # Extract data for sensor malfunction max magnitude plot
        if topic == '/bdetect/sensor_malfunction_max_magnitude':
            timestamps_sensor_malfunction.append(timestamp)
            sensor_malfunction_max_magnitude.append(msg.data[0])  # Assuming magnitude of the first sensor

# Create plots
fig, axs = plt.subplots(4, 1, figsize=(8, 20))

# Plot 1: Bird's eye view
axs[0].plot(x_positions, y_positions)
axs[0].set_xlabel('X Position')
axs[0].set_ylabel('Y Position')
axs[0].set_title('Bird\'s Eye View')
axs[0].grid(True)

# Plot 2: Ground truth and corrupted heading
axs[1].plot(timestamps_heading, ground_truth_heading, label='Ground Truth Heading')
axs[1].plot(timestamps_heading, corrupted_heading, label='Corrupted Heading')
axs[1].set_xlabel('Timestamp')
axs[1].set_ylabel('Heading (rad)')
axs[1].set_title('Ground Truth and Corrupted Heading')
axs[1].grid(True)
axs[1].legend()

axs[2].plot(timestamps_sensor_validity, bdetect_sensor_validity, label='bdetect/sensor_validity')
axs[2].plot(timestamps_sensor_validity, message_barrier_sensor_validity_final, label='message_barrier/sensor_validity_final')
axs[2].set_xlabel('Timestamp')
axs[2].set_ylabel('Sensor Validity')
axs[2].set_title('Sensor Validity')
axs[2].grid(True)
axs[2].legend()

axs[3].plot(timestamps_sensor_malfunction, sensor_malfunction_max_magnitude)
axs[3].set_xlabel('Timestamp')
axs[3].set_ylabel('Max Magnitude')
axs[3].set_title('Sensor Malfunction Max Magnitude')
axs[3].grid(True)

plt.tight_layout()
plt.show()
# %%
