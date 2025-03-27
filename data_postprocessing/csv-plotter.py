# %%
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import math

def get_notnull(df, column_name):
    return df[df[column_name].notnull()][column_name]

# %%
# BAG_NAME='2023-03-27-15-28-54_attacks-closed-loop-video-sync.bag'
# BAG_NAME='2023-03-27-15-41-01_attacks-open-loop-video-sync.bag'
BAG_NAME = "attacks-open-loop-video-sync_2023-03-27-15-41-01"
# Load the CSV file into a Pandas DataFrame
csv_file = Path(f'./{BAG_NAME}.csv')
df_raw = pd.read_csv(csv_file)
df = df_raw.copy()
df['timestamp_s'] = df['timestamp'] / 1e9
df['timestamp_relative_s'] = df['timestamp_s'] - df['timestamp_s'].iloc[0]
df.set_index('timestamp_relative_s', inplace=True)


# %%
# Define time range
start_time = 1679931126233426438 - 5*1e9
end_time = 1679931141993652013 + 5*1e9
# start_time = 1679931858134832916 - 5*1e9
# end_time = 1679931894540379184 + 5*1e9
start_time_relative_s = (start_time - df['timestamp'].iloc[0]) / 1e9
end_time_relative_s = (end_time - df['timestamp'].iloc[0]) / 1e9

# Filter the DataFrame based on the specified time range
df_filtered = df[(df.timestamp >= start_time) & (df.timestamp <= end_time)]

# %%
# Plot the data within the specified time range
# Plot 1: Bird's eye view, using x and y positions (filtered)
plt.figure()
plt.plot(df_filtered['x'], df_filtered['y'])
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Bird\'s Eye View')
plt.grid(True)
plt.axis('equal')

# %%
# Plot 2: Ground truth heading and corrupted heading (filtered)
fig, axes = plt.subplots(3,1)
ax = axes[0]
ax.step(get_notnull(df_filtered, 'heading_corruption').add(pd.Series({start_time_relative_s: 0.0, end_time_relative_s: 0.0}), fill_value=0), '--.', where='post', label='Corruption', color='tab:orange')
ax.set_ylabel('θ Offset (Δrad)')
ax.set_xlim(start_time_relative_s, end_time_relative_s)
ax.set_ylim(-math.pi, math.pi)
ax.grid(True)
ax.legend()
plt.setp(ax.get_xticklabels(), visible=False)

ax = axes[1]
ax.plot(get_notnull(df_filtered, 'ground_truth_heading'), '-o', label='Ground Truth')
ax.plot(get_notnull(df_filtered, 'corrupted_heading'), '--.', label='Corrupted')
ax.set_ylabel('θ (rad)')
ax.grid(True)
ax.legend()
ax.sharex(axes[0])
ax.sharey(axes[0])
plt.setp(ax.get_xticklabels(), visible=False)

# Plot 3: Sensor validity
ax = axes[2]
ax.plot(get_notnull(df_filtered, 'message_barrier_sensor_validity_final'), '--.', label='sensor_validity')
# ax.plot(get_notnull(df_filtered, 'bdetect_sensor_validity'), '-o', label='raw_sensor_validity')
ax.set_xlabel('Timestamp (s)')
ax.set_ylabel('Sensor Validity')
ax.grid(True)
ax.legend()
ax.sharex(axes[0])
fig.savefig('heading_corruption.png', dpi=300)


# %%
