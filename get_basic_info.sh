#!/bin/bash

# Saves some basic information about a system to log files
# Usage: ./get_basic_info.sh > get_basic_info.log 2>&1

set -o xtrace

hostname > hostname.log
uname -a > uname.log
sudo lshw > lshw.log
lsblk -o NAME,MODEL,SERIAL,SIZE,STATE > lsblk.log
df -h > df.log
ip a > ip.log
sudo lscpu > lscpu.log
sudo dmidecode -t memory > dmidecode-memory.log

rosversion -d > rosversion.log
rostopic list > rostopic-list.log
rosnode list > rosnode-list.log

mkdir -p rootfs/etc/
sudo cp -r /etc/ros rootfs/etc/

echo "Done!"
