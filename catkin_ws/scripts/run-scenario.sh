#!/bin/bash

set -o errexit -o nounset -o pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EXPERIMENTS_DIR="$SCRIPT_DIR"/../experiments

__experiment_id=$(date --iso-8601=seconds)
__experiment_suffix=${1:-experiment}
__experiment_dir="$EXPERIMENTS_DIR"/"$__experiment_id"_"$__experiment_suffix"

echo "Experiment ID: $__experiment_id"
echo "Experiment suffix: $__experiment_suffix"
echo "Experiment directory: $__experiment_dir"
mkdir -p "$__experiment_dir"

__ros_log_dir="$__experiment_dir"/ros-logs
mkdir -p "$__ros_log_dir"

export ROS_LOG_DIR="$__ros_log_dir"

# pipe stderr to stdout, add a timestamp using ts, then tee to a log file
ENABLE_EKF=false roslaunch --sigint-timeout=2 bcontrol sim.launch  2>&1 | ts | tee "$__experiment_dir"/ros-sim-launch.log &

sleep 3 # wait for roscore to start

DISPLAY=:1.0 roslaunch bcontrol visualization.launch 2>&1 | ts | tee "$__experiment_dir"/ros-visualization-launch.log &

rosrun bcontrol generate_detector_pipeline_launch_file.py $(rospack find bcontrol)/config/bdetect.yaml $(rospack find bcontrol)/launch/detector_pipeline.generated.launch 2>&1 | ts | tee "$__experiment_dir"/ros-generate-detector-pipeline-launch-file.log

roslaunch bcontrol stack.launch enable_detector:=false 2>&1 | ts | tee "$__experiment_dir"/ros-stack-launch.log &

# Wait for the duration of the experiment
sleep 5 # TODO: replace with an experiment script

echo "Experiment complete. Killing background processes..."
pkill -SIGINT -P $$ # $$ is the PID of the current process. pkill will send SIGINT to all child processes of the specified PID

wait

echo "All child processes exited."
