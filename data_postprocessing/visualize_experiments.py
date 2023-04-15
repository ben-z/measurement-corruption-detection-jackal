#%%
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import yaml
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent / "experiments"
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
# # turn the data into a dataframe
experiment_df = pd.json_normalize([
    {
        'config': config,
        'metrics': metrics or {},
        'has_metrics': metrics is not None,
        'bdetect_config': bdetect_config,
    }
    for config, metrics, bdetect_config in zip(experiment_configs, experiment_metrics, experiment_bdetect_config)
])
# Augment the dataframe with more attributes
experiment_df["bag_paths"] = [list(d.glob("*.bag")) for d in experiment_dirs]
print("Number of experiments:", len(experiment_df))
experiment_df_with_metrics = experiment_df[experiment_df.has_metrics]
print("Number of experiments with metrics:", len(experiment_df_with_metrics))

experiment_df.to_csv("experiment_df.csv")

# %%

# Further data processing
df = experiment_df_with_metrics

# %%
# Plot the effect of bias magnitude against detection result



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