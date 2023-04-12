#!/bin/bash

set -o errexit -o nounset -o pipefail

function slugify() {
    iconv -t ascii//TRANSLIT | sed -r s/[~\^]+//g | sed -r s/[^a-zA-Z0-9]+/-/g | sed -r s/^-+\|-+$//g | tr A-Z a-z
}

__machines_file=$(mktemp)
__commands_file=$(mktemp)
__joblog_file=logs/parallel-joblog-$(date --iso-8601=seconds | slugify).txt

# Populate the machines file
nodeattr -f tembo-genders.txt -n all > $__machines_file

echo "Using machines:"
cat $__machines_file

for i in {1..5}; do
    echo "echo \"Running command $i on machine \$(hostname) \$(date --iso-8601=seconds)\" | tee -a /tmp/test.txt; sleep 2; echo \"Finished command $i on machine \$(hostname) \$(date --iso-8601=seconds)\" | tee -a /tmp/test.txt" >> $__commands_file
done

echo "Running $(wc -l $__commands_file | awk '{print $1}') commands on $(wc -l $__machines_file | awk '{print $1}') machines..."
parallel --jobs 1 --joblog $__joblog_file --sshloginfile $__machines_file --workdir /tmp -a $__commands_file
