#!/bin/bash

set -o errexit -o nounset -o pipefail

source /etc/local.bashrc

__attack_start_delay=${1:-0} # seconds
__bias_magnitude=${2:-1.0} # radians

echo "Starting scenario with attack start delay: $__attack_start_delay and bias magnitude: $__bias_magnitude"

if [ -z "$EXPERIMENT_DIR" ]; then
    echo "EXPERIMENT_DIR not set. Exiting."
    exit 1
fi

if [[ $(type -t sleep_simtime) != "function" ]]; then
    echo "sleep_simtime not defined. Exiting."
    exit 1
fi

echo "Using experiment directory: $EXPERIMENT_DIR"

# Start visualization (for debugging)
# DISPLAY=:1.0 roslaunch bcontrol visualization.launch &

# Start rosbag recorder
unbuffer bash -c "$(declare -f rrecord); export -f rrecord; rrecord \"$EXPERIMENT_DIR/$EXPERIMENT_ID-$EXPERIMENT_SUFFIX.bag\" __name:=my_rosbag_recorder 2>&1 | ts | tee \"$EXPERIMENT_DIR\"/ros-record.log" &

echo "Waiting for steady state..."
sleep_simtime 15

# Start detector
echo "Starting detector..."
unbuffer roslaunch bcontrol detector.launch 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-detector-launch.log &
sleep_simtime 10 # wait for the detector to start

echo "Waiting for attack to start (delay: $__attack_start_delay)...)"
sleep_simtime "$__attack_start_delay"

echo "Performing attack..."
unbuffer rosrun bcontrol corruption_generator.py /global_localization/robot/odom/corruption nav_msgs/Odometry orientation step $__bias_magnitude __name:=my_corruption_generator 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-corruption-generator.log &
sleep_simtime 15

echo "Done performing attack. Waiting for recovery..."
unbuffer rosnode kill /my_corruption_generator 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-kill-corruption-generator.log
sleep_simtime 20

echo "Shutting down..."
unbuffer rosnode kill /my_rosbag_recorder 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-kill-rosbag-recorder.log || true
pkill -SIGINT -P $$ || true
wait

echo "Exiting..."