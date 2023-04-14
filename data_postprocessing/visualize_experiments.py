#%%
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import yaml
from pathlib import Path
import rosbags

SCRIPT_DIR = Path(__file__).parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent / "experiments"
print(f"{EXPERIMENTS_DIR=}")

# %%
# Ingest data

# Load the file list.
experiment_config_paths = list(EXPERIMENTS_DIR.glob("**/config.yaml"))
experiment_dirs = [config.parent for config in experiment_config_paths]
metric_paths = [d / "metrics.yaml" for d in experiment_dirs]

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

# %%
# # turn the data into a dataframe
experiment_df = pd.json_normalize([
    {
        'config': config,
        'metrics': metrics or {},
        'has_metrics': metrics is not None,
    }
    for config, metrics in zip(experiment_configs, experiment_metrics)
])
print("Number of experiments:", len(experiment_df))
experiment_df_with_metrics = experiment_df[experiment_df.has_metrics]
print("Number of experiments with metrics:", len(experiment_df_with_metrics))

experiment_df.to_csv("experiment_df.csv")

# %%

# Augment the dataframe with more attributes
# experiment_df["bag_paths"] = [list(d.glob("*.bag")) for d in experiment_dirs]



# %%
