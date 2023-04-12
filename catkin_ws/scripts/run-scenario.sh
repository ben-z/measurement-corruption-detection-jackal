#!/bin/bash

set -o errexit -o nounset -o pipefail

source /etc/local.bashrc

function __onexit() {
    echo "Killing background processes..."
    pkill -SIGINT -P $$ || true # $$ is the PID of the current process. pkill will send SIGINT to all child processes of the specified PID

    sleep 5

    echo "Killing any remaining background processes using SIGTERM..."
    pkill -SIGTERM -P $$ || true

    wait

    echo "All child processes exited."
}

trap __onexit EXIT

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EXPERIMENTS_DIR=${EXPERIMENTS_DIR:-/experiments}

export GAZEBO_WORLD=/workspace/gazebo-worlds/empty-rate_200.world
export REAL_TIME_FACTOR=0.2 # Need to be consistent with the real_time_update_rate in GAZEBO_WORLD
function sleep_simtime() {
    # Sleep for the specified number of seconds in simulation time
    sleep $(bc <<< "scale=2; $1 / $REAL_TIME_FACTOR")
}

__experiment_id=$(date --iso-8601=seconds | slugify)
__experiment_suffix=${1:-experiment}
__experiment_script=${2:-}
__experiment_dir="$EXPERIMENTS_DIR"/"$__experiment_id"_"$__experiment_suffix"

echo "Experiment ID: $__experiment_id"
echo "Experiment suffix: $__experiment_suffix"
echo "Experiment directory: $__experiment_dir"
echo "Experiment script: $__experiment_script"

if [ -z "$__experiment_script" ]; then
    echo "No script specified. Using default (sleep 10)."
    __experiment_script="sleep 10"
fi

mkdir -p "$__experiment_dir"

__ros_log_dir="$__experiment_dir"/ros-logs
mkdir -p "$__ros_log_dir"

export ROS_LOG_DIR="$__ros_log_dir"

# pipe stderr to stdout, add a timestamp using ts, then tee to a log file
ENABLE_EKF=false unbuffer roslaunch --sigint-timeout=2 bcontrol sim.launch enable_foxglove:=false gazebo_world:="$GAZEBO_WORLD" 2>&1 | ts | tee "$__experiment_dir"/ros-sim-launch.log &
sleep 3 # wait for roscore to start

# Generate launch files
unbuffer rosrun bcontrol generate_detector_pipeline_launch_file.py $(rospack find bcontrol)/config/bdetect.yaml $(rospack find bcontrol)/launch/detector_pipeline.generated.launch 2>&1 | ts | tee "$__experiment_dir"/ros-generate-detector-pipeline-launch-file.log

unbuffer roslaunch bcontrol stack.launch enable_detector:=false 2>&1 | ts | tee "$__experiment_dir"/ros-stack-launch.log &
sleep 5 # wait for the stack to start

# Scenario =======================
export EXPERIMENT_DIR="$__experiment_dir"
export EXPERIMENT_ID="$__experiment_id"
export EXPERIMENT_SUFFIX="$__experiment_suffix"

echo "Running experiment script: $__experiment_script"
# declare -f sleep_simtime will print the definition of the sleep_simtime function so that it can be used in the experiment script
unbuffer bash -c "$(declare -f sleep_simtime); export -f sleep_simtime; $__experiment_script" 2>&1 | ts | tee "$__experiment_dir"/experiment-script.log
