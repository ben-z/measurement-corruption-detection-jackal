#!/bin/bash

set -o errexit -o nounset -o pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run the tests
for test_file in \
    "$SCRIPT_DIR"/catkin_ws/src/bcontrol/src/*_test.py \
    "$SCRIPT_DIR"/catkin_ws/src/bcontrol/src/utils.py \
; do
    echo "Running $test_file"
    python3 "$test_file"
    __exit_code=$?
    echo "$test_file completed with exit code $__exit_code"
done

