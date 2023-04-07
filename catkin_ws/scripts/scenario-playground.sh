#!/bin/bash

set -o errexit -o nounset -o pipefail

source /etc/local.bashrc

echo "Starting scenario..."

if [ -z "$EXPERIMENT_DIR" ]; then
    echo "EXPERIMENT_DIR not set. Exiting."
    exit 1
fi

echo "Using experiment directory: $EXPERIMENT_DIR"

# Start visualization (for debugging)
DISPLAY=:1.0 roslaunch bcontrol visualization.launch &
# Start rosbag recorder
rrecord "$EXPERIMENT_DIR/$EXPERIMENT_ID-$EXPERIMENT_SUFFIX.bag" __name:=my_rosbag_recorder 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-record.log &

echo "Waiting for steady state..."
sleep 15

# Start detector
roslaunch bcontrol detector.launch 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-detector-launch.log &

sleep 10 # wait for the detector to start

echo "Performing attack"
./src/bcontrol/src/corruption_generator.py /global_localization/robot/odom/corruption nav_msgs/Odometry orientation step -0.5 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-corruption-generator.log &
__gen_pid=$!

sleep 10
kill -SIGINT $__gen_pid

echo "Done performing attack. Waiting for recovery..."
sleep 20

echo "Shutting down..."
rosnode kill /my_rosbag_recorder 2>&1 | ts | tee "$EXPERIMENT_DIR"/ros-kill-rosbag-recorder.log || true
pkill -SIGINT -P $$ || true
wait

echo "Exiting..."