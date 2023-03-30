#!/bin/bash

set -o errexit -o nounset -o pipefail -o xtrace

# Run the tests
python3 ./src/bcontrol/src/utils.py