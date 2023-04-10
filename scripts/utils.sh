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