#!/bin/bash
{ # This ensures the entire script is read into memory before execution. Ref: https://stackoverflow.com/a/2358432/4527337

# This script runs the experiment with varying detector_solve_hz values.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR"/utils.sh

set -o errexit -o nounset -o pipefail

__machines_file=$(mktemp)
__commands_file=$(mktemp)
__joblog_file=logs/parallel-joblog-$(date --iso-8601=seconds | slugify).txt

__node_group=${1:-all}

__experiments_dir=./experiments
if [ ! -d "$__experiments_dir" ]; then
    2>&1 echo "Missing experiments directory '$__experiments_dir'"
    exit 1
fi

# Populate the machines file
nodeattr -f tembo-genders.txt -n $__node_group > $__machines_file

echo "Using machines:"
cat $__machines_file


# Populate the commands file

bias=-1.0
delay=1.0
# for detector_solve_hz in $(loop_with_step 1.0 20.0 1.0); do
for detector_solve_hz in $(loop_with_step 0.1 1.0 0.1); do
    __exp_name_prefix="detector-solve-hz-$detector_solve_hz"

    # if the experiment is already done, skip it
    __existing_experiments=$(find_existing_experiments "$__experiments_dir" "$__exp_name_prefix")
    if [ -n "$__existing_experiments" ]; then
        echo "Skipping experiment '$__exp_name_prefix' because it already exists as $__existing_experiments"
        continue
    fi

    generate_tembo_scenario \
        --experiment_name "$__exp_name_prefix-\$(hostname)" \
        --gazebo_world "empty-rate_100" \
        corruption /global_localization/robot/odom/corruption nav_msgs/Odometry orientation step $bias --corruption_start_sec $delay \
        --detector_solve_hz $detector_solve_hz \
    >> $__commands_file
done

echo "Running $(wc -l $__commands_file | awk '{print $1}') command(s) from '$__commands_file' on $(wc -l $__machines_file | awk '{print $1}') machine(s)..."
sleep 5

# For debugging
# parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --line-buffer -a $__commands_file
# For monitoring progress
# parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --progress -a $__commands_file

exit
}