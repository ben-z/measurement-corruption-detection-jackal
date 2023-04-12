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
    __scenario_name=$1
    __scenario_command=$2

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
            && sudo chown -R \\\$(id -u):\\\$(id -g) /workspace /experiments
            && rsync -a /workspace{_ro/catkin_ws,/.}
            && mkdir -p \\\$EXPERIMENTS_DIR
            && rm -rf build install logs
            && source /etc/local.bashrc
            && catkin build
            && ./scripts/run-scenario.sh $__scenario_name '$__scenario_command'
            ; if [ -d \\\$EXPERIMENTS_DIR ]; then sudo rsync -a \\\$EXPERIMENTS_DIR/. /experiments; fi
        "
EOF
    )
#     echo $(cat <<-EOF
#         bash -c "echo '${__scenario_name}' && bash -c '${__scenario_command}'"
# EOF
#     )
}

# Construct the commands file
# for bias in $(loop_with_step -2.0 2.0 5.0); do
for bias in $(loop_with_step -2.0 2.0 0.1); do
    # for delay in $(loop_with_step 0.0 20.0 21.0); do
    for delay in $(loop_with_step 0.0 20.0 0.5); do
        generate_scenario "heading-bias-$bias-with-delay-$delay-\$(hostname)" "./scripts/scenario-playground.sh $delay $bias" >> $__commands_file
        # generate_scenario "heading-bias-$bias-with-delay-$delay-\$(hostname)" "" >> $__commands_file
        # generate_scenario "heading-bias-$bias-with-delay-$delay-\$(hostname)" 'echo \"Done running on $(hostname)\"' >> $__commands_file
    done
done

echo "Running $(wc -l $__commands_file | awk '{print $1}') command(s) on $(wc -l $__machines_file | awk '{print $1}') machine(s)..."
# For debugging
# parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --line-buffer -a $__commands_file
# For monitoring progress
parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --progress -a $__commands_file
