#!/bin/bash
{ # This ensures the entire script is read into memory before execution. Ref: https://stackoverflow.com/a/2358432/4527337

# This script finds all rosbags and extracts the metrics from them.

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

function generate_extract_command() {
    # EXPERIMENTS_DIR contains the results of the experiment.
    # We first store it in /workspace (a tmpfs mount) and then rsync it to /experiments
    # at the end of the experiment (a persistent NFS mount).
    # This eliminates any network-related issues.

    echo $(cat <<-EOF
        source \$HOME/benz/research-jackal/tembo-bashrc
        && cd ~/benz/research-jackal
        && conda activate jackal
        && python ./data_postprocessing/rosbag-augmenter.py $@
EOF
    )
}

# Populate the commands file
for bag_path in $(find $__experiments_dir -name '*.bag'); do
    if [ ! -f "$(dirname $bag_path)/config.yaml" ]; then
        echo "Skipping '$bag_path' because it doesn't have a config.yaml file"
        continue
    fi
    generate_extract_command $bag_path >> $__commands_file
done

echo "Running $(wc -l $__commands_file | awk '{print $1}') command(s) from '$__commands_file' on $(wc -l $__machines_file | awk '{print $1}') machine(s)..."

# For debugging
# parallel --retries 2 --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --line-buffer -a $__commands_file
# For monitoring progress
sleep 5
parallel --verbose --retries 2 --jobs 24 --joblog $__joblog_file --sshloginfile $__machines_file --workdir $(pwd) --progress -a $__commands_file

exit
}