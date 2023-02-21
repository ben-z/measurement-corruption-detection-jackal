# Jackal Research

## Workspace setup

Please use `Dockerfile` as the source of truth for required packages. The following instructions are for quick reference only.

### ROS

1. Install ROS Noetic.
1. Install catkin_tools: `pip install catkin_tools`
1. Install Jackal packages: `sudo apt install ros-noetic-jackal-*`

### Development tools

```bash
sudo apt install -y \
    grepcidr \ # CIDR range matching (for use with scripts that automatically connect to robots)
    ripgrep
```

## Robot setup

### Jackal2 Configuration

`ben_jackal2.vsk`: VICON configuration for the Jackal2 object. This contains 3 markers around the body and 3 markers surrounding the IMU unit.

### Connecting the controller to the robot

Put the controller in pairing mode. On the controller, press the PS button and the Share button at the same time. The controller will start flashing.

```bash
sudo bluetoothctl
scan on
# wait for the controller to show up
scan off
pair <controller-mac-address> # connect to the controller
trust <controller-mac-address> # trust the controller
connect <controller-mac-address> # connect to the controller
```

Troubleshooting tips:
- https://superuser.com/a/1199595/465482

### Connecting rviz to the robot

```bash
source /opt/ros/noetic/setup.sh
export ROS_MASTER_URI=http://jackal1.robohub.eng.uwaterloo.ca:11311
```

### Recording rosbags on the robot

```bash
# SSH into the robot
ssh jackal1

# Record. -j enables compression, -a records all topics,
# -x excludes some topics, -o adds a prefix to the file name
rosbag record -j -a -x "(.*)/camera(.*)" -o <recording-prefix>
```

## Simulation

Start the Docker container:

```bash
./generate-dot-env.sh
docker compose up -d --build
docker compose exec dev bash
```

All commands from now on should be run inside the container.

cd into the catkin workspace:

```bash
devsetup # shortcut (defined in /etc/local.bashrc)
```

Install dependencies:

```bash
rosdep update
rosdep_install_all # shortcut (defined in /etc/local.bashrc)
```

Build the workspace:

```bash
catkin build
```

Start the simulation and the stack:

```bash
# Environmental variable configuration reference: https://github.com/jackal/jackal/commit/75c68523945b0edf504c64b8b188260292bb3472
export JACKAL_LASER_3D=1 # Enable 3D laser
# Launch the simulation
DISPLAY=:1.0 roslaunch bcontrol sim.launch
# Launch the stack
roslaunch bcontrol stack.launch
```

## Development notes

`cmd_vel` is the topic for sending velocity commands to the robot. The message type is `geometry_msgs/Twist`.

`/opt/ros/noetic/share/jackal_control/launch/control.launch` launches the EKF when the `enable_ekf` parameter or the `ENABLE_EKF` environment variable is true. Defaults to true. `/opt/ros/noetic/share/jackal_control/config/robot_localization.yaml` is the configuration for the EKF.

A graph of nodes and topics:

![rosgraph](./docs/rosgraph.png)