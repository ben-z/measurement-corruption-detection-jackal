#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR"/utils.sh

set -o errexit -o nounset -o pipefail

__gender=$1

for bias in $(loop_with_step -2.0 2.0 0.1); do
  echo "current bias: $bias"

  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose down'
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose up -d --build sim_headless'
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose exec sim_headless bash -c "sudo rsync -a /workspace{_ro/catkin_ws,/.} && rm -rf build install logs && source /etc/local.bashrc && catkin build"'
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER "cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose exec sim_headless bash -c \"sudo rsync -a /workspace{_ro/catkin_ws,/.} && source /etc/local.bashrc && catkin build && ./scripts/run-scenario.sh $(hostname)-heading-bias-$bias \\\"./scripts/scenario-playground.sh 0.0 $bias\\\"\""
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose down'
done