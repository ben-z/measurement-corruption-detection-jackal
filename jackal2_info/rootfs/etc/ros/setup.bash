# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######

export JACKAL_FLEA3=1
export JACKAL_FLEA3_TILT=0
export JACKAL_FLEA3_OFFSET="0 0 0.005"
export JACKAL_MAG_CONFIG=/home/administrator/catkin_ws/src/wat65_jackal/urdf/mag_config.urdf.xacro

export JACKAL_URDF_EXTRAS=/home/administrator/catkin_ws/src/wat65_jackal/urdf/jackal_description.urdf.xacro

# Pass through to the main ROS workspace of the system.
#source /opt/ros/kinetic/setup.bash
source /home/administrator/catkin_ws/devel/setup.bash

