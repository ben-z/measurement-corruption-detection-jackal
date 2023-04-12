#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR"/utils.sh

set -o errexit -o nounset -o pipefail

__gender=$1

__bias=-1.0

for delay in $(loop_with_step 0.0 20.0 0.5); do
  echo "current delay: $delay"

  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose down'
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose up -d --build sim_headless'
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose exec sim_headless bash -c "sudo rsync -a /workspace{_ro/catkin_ws,/.} && rm -rf build install logs && source /etc/local.bashrc && catkin build"'
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER "cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose exec sim_headless bash -c \"sudo rsync -a /workspace{_ro/catkin_ws,/.} && source /etc/local.bashrc && catkin build && ./scripts/run-scenario.sh \$(hostname)-heading-bias-$__bias-with-delay-$delay \\\"./scripts/scenario-playground.sh $delay $__bias\\\"\""
  pdsh -S -F ./tembo-genders.txt -g $__gender -l $USER 'cd ~/benz/research-jackal && source /hdd2/.host_profile && docker compose down'
done
