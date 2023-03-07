#!/usr/bin/env python3

import sys

# This file is used by the launch file to terminate the launch

error_message = sys.argv[1] if len(sys.argv) > 1 else "No error message provided"

print(f"{sys.argv[0]} ERROR: {error_message}")
print(f"{sys.argv[0]} Terminating...")

sys.exit(1)