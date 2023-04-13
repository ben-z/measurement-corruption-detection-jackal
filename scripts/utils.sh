#!/bin/bash

loop_with_step() {
  # Generates a sequence of values from start to end with a given step size
  # using the bc command-line calculator.
  # Example:
  # for v in $(loop_with_step 1.0 10.0 0.1); do
  #   echo "Current value is: $v"
  # done

  # Get input arguments
  local start_value="$1"
  local end_value="$2"
  local step_size="$3"

  # Initialize the loop variable
  local value="$start_value"

  # Initialize the result string
  local result=""

  # Generate the sequence
  while (( $(echo "$value <= $end_value" | bc -l) )); do
    # Append the current value to the result string
    result+="$value "

    # Increment the loop variable
    value=$(echo "$value + $step_size" | bc -l)
  done

  # Output the generated sequence
  echo "$result"
}

function slugify() {
    iconv -t ascii//TRANSLIT | sed -r s/[~\^]+//g | sed -r s/[^a-zA-Z0-9]+/-/g | sed -r s/^-+\|-+$//g | tr A-Z a-z
}


function generate_tembo_scenario() {
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

function find_existing_experiments() {
  __experiments_dir="$1"
  __exp_name_prefix="$2"

  date_slug_pattern="[0-9]{4}-[0-9]{2}-[0-9]{2}t[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{6}"

  find "$__experiments_dir" -maxdepth 1 -name "*$__exp_name_prefix*" -type d | grep -E "/${date_slug_pattern}-${__exp_name_prefix}.*\$" || true
}