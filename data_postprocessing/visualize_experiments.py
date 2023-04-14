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

# read the config files
experiment_configs = []
for config_path in experiment_config_paths:
    with open(config_path, "r") as f:
        experiment_configs.append(yaml.safe_load(f))

# %%
# # turn the config files into a dataframe
experiment_df = pd.json_normalize(experiment_configs)

# %%

# Augment the dataframe with more attributes
experiment_df["bag_paths"] = [list(d.glob("*.bag")) for d in experiment_dirs]



# %%
