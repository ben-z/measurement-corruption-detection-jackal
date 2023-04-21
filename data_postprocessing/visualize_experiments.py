#%%
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import yaml
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
# tembo
# EXPERIMENTS_DIR = SCRIPT_DIR.parent / "experiments"
# wato, scratch
EXPERIMENTS_DIR = Path('/mnt/scratch/ben-tembo-experiments/tembo-experiments')
print(f"{EXPERIMENTS_DIR=}")

# %%
# Ingest data

# Load the file list.
experiment_config_paths = list(EXPERIMENTS_DIR.glob("**/config.yaml"))
experiment_dirs = [config.parent for config in experiment_config_paths]
metric_paths = [d / "metrics.yaml" for d in experiment_dirs]
bdetect_config_paths = [d / "bdetect.yaml" for d in experiment_dirs]

# read the config files
experiment_configs = []
for config_path in experiment_config_paths:
    with open(config_path, "r") as f:
        experiment_configs.append(yaml.safe_load(f))

# read the metric files
experiment_metrics = []
for metric_path in metric_paths:
    if not metric_path.exists():
        experiment_metrics.append(None)
        continue
    with open(metric_path, "r") as f:
        experiment_metrics.append(yaml.safe_load(f))

experiment_bdetect_config = []
for bdetect_config_path in bdetect_config_paths:
    with open(bdetect_config_path, "r") as f:
        experiment_bdetect_config.append(yaml.safe_load(f))

# %%
# turn the data into a dataframe
experiment_df = pd.json_normalize([
    {
        'experiment_dir': experiment_dir,
        'config': config,
        'metrics': metrics or {},
        'has_metrics': metrics is not None,
        'bdetect_config': bdetect_config,
    }
    for (
        experiment_dir, \
        config, \
        metrics, \
        bdetect_config, \
    ) in zip(
        experiment_dirs, \
        experiment_configs, \
        experiment_metrics, \
        experiment_bdetect_config, \
    )
])
# Augment the dataframe with more attributes
experiment_df["bag_paths"] = [list(d.glob("*.bag")) for d in experiment_dirs]
print("Number of experiments:", len(experiment_df))
experiment_df_with_metrics = experiment_df[experiment_df.has_metrics]
print("Number of experiments with metrics:", len(experiment_df_with_metrics))


# %%

def plot_distribution(df, column, figsize=(10, 6)):
    plt.figure(figsize=figsize)
    sns.countplot(data=df, x=column)
    plt.title(f'Distribution of {column}')
    plt.xlabel(column)
    plt.ylabel('Count')
    plt.xticks(rotation=45, ha='right')
    plt.show()

# Preprocess the data
df = experiment_df_with_metrics
# populate the planner path profile with default values
default_planner_path_profile = "type=circle,radius=2"
df['config.planner_path_profile'] = df['config.planner_path_profile'].apply(lambda x: x or default_planner_path_profile)
df['config.planner_path_profile'].fillna(default_planner_path_profile, inplace=True)
plot_distribution(df, 'config.planner_path_profile')

# populate the detector solve hz with default values
df['config.scenario_config.args.detector_solve_hz'].fillna(20, inplace=True)
plot_distribution(df, 'config.scenario_config.args.detector_solve_hz')

# Populate config.detector_window_size with default values
default_window_size = 80
df['config.detector_window_size'] = df['config.detector_window_size'].apply(lambda x: x or default_window_size)
df['config.detector_window_size'].fillna(default_window_size, inplace=True)
plot_distribution(df, 'config.detector_window_size')

start_without_init = df['metrics.corruption_init_ns'].isna() & ~df['metrics.corruption_start_ns'].isna()
print(f"Removing {start_without_init.sum()} experiments that have corruption_start_ns but not corruption_init_ns")
# remove these experiments until we figure out what's wrong. Perhaps these are older experiments where
# we didn't have latching?
df = df[~start_without_init]

# Find the rows without metrics.corruption_init_ns
no_init = df['metrics.corruption_init_ns'].isna()
assert no_init.sum() == 0, "Some experiments have no corruptions"

no_start = df['metrics.corruption_start_ns'].isna()
print(f"Removing {no_start.sum()} experiments that have no corruption_start_ns")
df = df[~no_start]

no_stop = df['metrics.corruption_stop_ns'].isna()
assert no_stop.sum() == 0, "Some experiments have no corruption_stop_ns"

columns_with_only_nan = df.isna().all()
columns_with_only_nan = columns_with_only_nan[columns_with_only_nan]
print("Removing the following columns because they are all NaN:")
print(columns_with_only_nan)
df = df.drop(columns_with_only_nan.index, axis=1)

print("Columns with NaN values:")
columns_with_nan = df.isna().any()
columns_with_nan[columns_with_nan]

df.to_csv("experiment_df.csv")
df_clean = df

# %%
# Plot the effect of bias magnitude against detection result
df = df_clean
print(f"Loaded {len(df)} experiments")

# Keep only bias experiments
df = df[df['config.scenario_config.args.signal_type'] == 'step']

# Only keep paths that are circles of radius 2
df = df[df['config.planner_path_profile'] == 'type=circle,radius=2']

# Only keep heading corruptions
df = df[df['metrics.corruption_idx'] == 2]

print(f"Using {len(df)} experiments")

# Plot the distribution of data
plot_distribution(df, 'config.planner_path_profile')
plot_distribution(df, 'metrics.corruption_idx')
plot_distribution(df, 'config.scenario_config.args.magnitude')

# Extract relevant columns
relevant_df = df[['metrics.detection_result', 'config.scenario_config.args.magnitude']]

# Group by bias magnitude, calculate the detection rate and count the number of data points for each magnitude
grouped_df_with_count = relevant_df.groupby('config.scenario_config.args.magnitude').agg(
    detection_rate=('metrics.detection_result', lambda x: (x == 'detected').mean() * 100),
    count=('metrics.detection_result', 'count')
).reset_index()

# Remove magnitudes with small number of data points
filtered_df = grouped_df_with_count[grouped_df_with_count['count'] >= 3]

# Plot the detection rate vs bias magnitude
plt.figure(figsize=(10, 6))
plt.plot(filtered_df['config.scenario_config.args.magnitude'], filtered_df['detection_rate'], marker='o')
plt.xlabel('Bias Magnitude')
plt.ylabel('Detection Rate (%)')
plt.title('Detection Rate vs. Bias Magnitude (Filtered)')
plt.grid(True)
plt.show()

df.to_csv("detection_rate_vs_bias_magnitude_raw.csv")

# %%
df = experiment_df_with_metrics

print(df.columns)

# Keep only experiments that don't have a special path profile
df = df[df['config.planner_path_profile'] == ""]

# Keep only data with the default solve hz
df = df[df['config.scenario_config.args.detector_solve_hz'].isna()]

# Keep data with no metrics errors
df = df[df['metrics.errors'].apply(lambda x: len(x) == 0)]

# Remove df with no detections
# df = df[df['metrics.detection_result'] != "not detected"]
# assert all(~df['metrics.detection_ns'].isna())
# assert all(~df['metrics.first_sensor_measurement_after_corruption_ns'].isna())

normalization_df = df['metrics.first_sensor_measurement_after_corruption_ns']

df['normalized_detection_s'] = (df['metrics.detection_ns'] - normalization_df) / 1e9
df['normalized_detection_s'].describe()
# print(df['metrics.first_detector_output_after_sensor_measurement_ns'].isna().sum())
df['normalized_first_detector_output_s'] = (df['metrics.first_detector_output_after_sensor_measurement_ns'] - normalization_df) / 1e9

plt.figure(figsize=(10, 6))
sns.stripplot(data=df, x='normalized_first_detector_output_s', y='metrics.detection_result', jitter=True, alpha=0.5)
plt.title('Relationship between First Detector Output Time and Detection Result')
plt.xlabel('First Detector Output after Sensor COrruption (s)')
plt.ylabel('Detection Result')
plt.xlim(-0.1, 0.5)
plt.show()


# %%

df = experiment_df_with_metrics

print(df.columns)

# Keep only step attacks
df = df[df['config.scenario_config.args.signal_type'] == "step"]

# Keep only data with special detector solve hz
# df = df[~df['config.scenario_config.args.detector_solve_hz'].isna()]

df['config.scenario_config.args.detector_solve_hz'].fillna(20, inplace=True)
assert all(~df['config.scenario_config.args.detector_solve_hz'].isna())

# # Keep data with no metrics errors
# df = df[df['metrics.errors'].apply(lambda x: len(x) == 0)]

# Remove df with no detections
# df = df[df['metrics.detection_result'] != "not detected"]
# assert all(~df['metrics.detection_ns'].isna())
# assert all(~df['metrics.first_sensor_measurement_after_corruption_ns'].isna())

normalization_df = df['metrics.first_sensor_measurement_after_corruption_ns']

df['normalized_detection_s'] = (df['metrics.detection_ns'] - normalization_df) / 1e9
df['normalized_detection_s'].describe()
# print(df['metrics.first_detector_output_after_sensor_measurement_ns'].isna().sum())
df['normalized_first_detector_output_s'] = (df['metrics.first_detector_output_after_sensor_measurement_ns'] - normalization_df) / 1e9

plt.figure(figsize=(10, 6))
sns.stripplot(data=df, x='config.scenario_config.args.detector_solve_hz', y='metrics.detection_result', jitter=True, alpha=0.5)
plt.title('Relationship between detector_solve_hz and Detection Result')
plt.xlabel('detector_solve_hz')
plt.ylabel('Detection Result')
# plt.xlim(0, 1)
plt.show()
# %%

df = experiment_df_with_metrics

# Keep only step attacks
df = df[df['config.scenario_config.args.signal_type'] == "step"]

df['config.scenario_config.args.detector_solve_hz'].fillna(20, inplace=True)
assert all(~df['config.scenario_config.args.detector_solve_hz'].isna())

# # Keep data with no metrics errors
# df = df[df['metrics.errors'].apply(lambda x: len(x) == 0)]

# Remove df with no detections
# df = df[df['metrics.detection_result'] != "not detected"]
# assert all(~df['metrics.detection_ns'].isna())
# assert all(~df['metrics.first_sensor_measurement_after_corruption_ns'].isna())

normalization_df = df['metrics.first_sensor_measurement_after_corruption_ns']

df['normalized_detection_s'] = (df['metrics.detection_ns'] - normalization_df) / 1e9
df['normalized_detection_s'].describe()
# print(df['metrics.first_detector_output_after_sensor_measurement_ns'].isna().sum())
df['normalized_first_detector_output_s'] = (df['metrics.first_detector_output_after_sensor_measurement_ns'] - normalization_df) / 1e9

plt.figure(figsize=(10, 6))
sns.stripplot(data=df, x='config.scenario_config.args.detector_solve_hz', y='metrics.detection_result', jitter=True, alpha=0.5)
plt.title('Relationship between detector_solve_hz and Detection Result')
plt.xlabel('detector_solve_hz')
plt.ylabel('Detection Result')
plt.xlim(0, 1)
plt.show()
# %%
