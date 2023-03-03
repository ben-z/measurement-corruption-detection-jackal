#!/bin/bash -e

# This script is a wrapper for TurboVNC's vncserver script. It handles SIGINT properly.

# Path to vncserver script
VNC_SERVER=/opt/TurboVNC/bin/vncserver

if [ -z "$DISPLAY" ]; then
    2>&1 echo "DISPLAY environment variable is not set. Please set it to the display number you want to use. For example: DISPLAY=:1"
    exit 1
fi

# Listen to SIGINT
trap "echo 'Received SIGINT. Shutting down TurboVNC server...'; $VNC_SERVER -kill $DISPLAY; echo Done; exit 0" SIGINT

# Enable xtrace
set -o xtrace

# Start vncserver
$VNC_SERVER $@ &
VNC_SERVER_PID=$!

# Disable xtrace
{ set +o xtrace; } 2>/dev/null

wait $VNC_SERVER_PID