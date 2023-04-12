#!/bin/bash

set -o errexit -o nounset -o pipefail

__machines_file=$(mktemp)
__commands_file=$(mktemp)

# Populate the machines file
nodeattr -f tembo-genders.txt -n all > $__machines_file

echo "Using machines:"
cat $__machines_file

for i in {1..5}; do
    echo "echo \"Running command $i on machine \$(hostname) \$(date --iso-8601=seconds)\" | tee -a /tmp/test.txt; sleep 2; echo \"Finished command $i on machine \$(hostname) \$(date --iso-8601=seconds)\" | tee -a /tmp/test.txt" >> $__commands_file
done

echo "Running the following commands:"
cat $__commands_file

echo "Running commands in parallel"
parallel --jobs 1 --sshloginfile $__machines_file --workdir /tmp -a $__commands_file
