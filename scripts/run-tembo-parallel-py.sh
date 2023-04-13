#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR"/utils.sh

set -o errexit -o nounset -o pipefail

__machines_file=$(mktemp)
__commands_file=$(mktemp)
__joblog_file=logs/parallel-joblog-$(date --iso-8601=seconds | slugify).txt

# Populate the machines file
nodeattr -f tembo-genders.txt -n all > $__machines_file

echo "Using machines:"
cat $__machines_file

function generate_scenario() {
    __run_experiment_args=$@

    # EXPERIMENTS_DIR contains the results of the experiment.
    # We first store it in /workspace (a tmpfs mount) and then rsync it to /experiments
    # at the end of the experiment (a persistent NFS mount).
    # This eliminates any network-related issues.

    echo $(cat <<-EOF
        cd ~/benz/research-jackal
        && ./scripts/bootstrap-tembo.sh
        && source /hdd2/.host_profile
        && docker compose down
        && docker compose up -d --build sim_headless
        && docker compose exec sim_headless bash -c "
            export EXPERIMENTS_DIR=/workspace/experiments_tmp
            && sudo chown \\\$(id -u):\\\$(id -g) /workspace /experiments
            && rsync -a /workspace{_ro/catkin_ws,_ro/gazebo-worlds,/.}
            && mkdir -p \\\$EXPERIMENTS_DIR
            && source /etc/local.bashrc
            && catkin build
            && python3 -m scripts.run_experiment --experiments_dir \\\$EXPERIMENTS_DIR $__run_experiment_args
            ; if [ -d \\\$EXPERIMENTS_DIR ]; then sudo rsync -a \\\$EXPERIMENTS_DIR/. /experiments; fi
        "
EOF
    )
}

# Construct the commands file

bias=-1.0
delay=0.0
for detector_solve_hz in $(loop_with_step 1.0 20.0 1.0); do
    generate_scenario \
        --experiment_name "detector-solve-hz-$detector_solve_hz-\$(hostname)" \
        --gazebo_world "empty-rate_100" \
        corruption /global_localization/robot/odom/corruption nav_msgs/Odometry orientation step $bias --corruption_start_sec $delay \
        --detector_solve_hz $detector_solve_hz \
    >> $__commands_file
done

for path_profile in \
    "type=circle,radius=2" \
    "type=circle,radius=5" \
    "type=circle,radius=1" \
    "type=figure_eight,length=10,width=5" \
    "type=figure_eight,length=5,width=2" \
    "type=figure_eight,length=20,width=10" \
; do
    for bias in $(loop_with_step -1.5 1.5 0.2); do
        for delay in $(loop_with_step 2.0 20.0 4.0); do
            generate_scenario \
                --experiment_name "heading-bias-$bias-with-delay-$delay-\$(hostname)" \
                --gazebo_world "empty-rate_200" \
                --path_profile "$path_profile" \
                corruption /global_localization/robot/odom/corruption nav_msgs/Odometry orientation step $bias --corruption_start_sec $delay \
            >> $__commands_file
        done
    done
done


echo "Running $(wc -l $__commands_file | awk '{print $1}') command(s) from '$__commands_file' on $(wc -l $__machines_file | awk '{print $1}') machine(s)..."
# For debugging
# parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --line-buffer -a $__commands_file
# For monitoring progress
parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --progress -a $__commands_file
