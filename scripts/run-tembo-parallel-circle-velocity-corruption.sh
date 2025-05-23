#!/bin/bash
{ # This ensures the entire script is read into memory before execution. Ref: https://stackoverflow.com/a/2358432/4527337

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

for planner_path_profile in \
    "type=circle,radius=0.5" \
    "type=circle,radius=1" \
    "type=circle,radius=2" \
    "type=circle,radius=5" \
    "type=circle,radius=10" \
; do
    for magnitude in $(loop_with_step -2.0 2.0 0.2); do
        # step
        __exp_name_prefix="velocity-corruption-bias-$magnitude-$(echo "$planner_path_profile" | slugify)"

        # if the experiment is already done, skip it
        __existing_experiments=$(find_existing_experiments "$__experiments_dir" "$__exp_name_prefix")
        if [ -n "$__existing_experiments" ]; then
            echo "Skipping experiment '$__exp_name_prefix' because it already exists as $__existing_experiments"
        else
            generate_tembo_scenario \
                --experiment_name "$__exp_name_prefix-\$(hostname)" \
                --gazebo_world "empty-rate_200" \
                corruption /jackal_velocity_controller/odom/corruption nav_msgs/Odometry linear_vel_x step $magnitude \
            >> $__commands_file
        fi

        # ramp
        __exp_name_prefix="velocity-corruption-ramp-$magnitude-$(echo "$planner_path_profile" | slugify)"

        # if the experiment is already done, skip it
        __existing_experiments=$(find_existing_experiments "$__experiments_dir" "$__exp_name_prefix")
        if [ -n "$__existing_experiments" ]; then
            echo "Skipping experiment '$__exp_name_prefix' because it already exists as $__existing_experiments"
        else
            generate_tembo_scenario \
                --experiment_name "$__exp_name_prefix-\$(hostname)" \
                --gazebo_world "empty-rate_200" \
                corruption /jackal_velocity_controller/odom/corruption nav_msgs/Odometry linear_vel_x ramp $magnitude \
            >> $__commands_file
        fi

        # oscillating
        for period in $(loop_with_step -10.0 10.0 1.0); do
            __exp_name_prefix="velocity-corruption-oscillation-$magnitude-$period-$(echo "$planner_path_profile" | slugify)"

            # if the experiment is already done, skip it
            __existing_experiments=$(find_existing_experiments "$__experiments_dir" "$__exp_name_prefix")
            if [ -n "$__existing_experiments" ]; then
                echo "Skipping experiment '$__exp_name_prefix' because it already exists as $__existing_experiments"
            else
                generate_tembo_scenario \
                    --experiment_name "$__exp_name_prefix-\$(hostname)" \
                    --gazebo_world "empty-rate_200" \
                    corruption /jackal_velocity_controller/odom/corruption nav_msgs/Odometry linear_vel_x oscillating $magnitude --period $period \
                >> $__commands_file
            fi
        done
    done
done

echo "Running $(wc -l $__commands_file | awk '{print $1}') command(s) from '$__commands_file' on $(wc -l $__machines_file | awk '{print $1}') machine(s)..."

# For debugging
# parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --line-buffer -a $__commands_file
# For monitoring progress
sleep 5
parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --progress -a $__commands_file

exit
}