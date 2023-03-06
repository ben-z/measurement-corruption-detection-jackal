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
devsetup # shortcut for cd-ing into the catkin_ws dir and sourcing setup.bash (defined in /etc/local.bashrc)
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
VGL_DISPLAY=egl0 DISPLAY=:1.0 vglrun roslaunch bcontrol sim.launch
# Launch the stack
roslaunch bcontrol stack.launch
```

`VGL_DISPLAY=egl0` is used to tell VirtualGL to use the first EGL device. Each EGL device corresponds to a `/dev/dri/card*` device.

Useful VGL commands:

```bash
sudo /opt/VirtualGL/bin/eglinfo -e # list available EGL devices
```

## Development notes

`cmd_vel` is the topic for sending velocity commands to the robot. The message type is `geometry_msgs/Twist`.

`/opt/ros/noetic/share/jackal_control/launch/control.launch` launches the EKF when the `enable_ekf` parameter or the `ENABLE_EKF` environment variable is true. Defaults to true. `/opt/ros/noetic/share/jackal_control/config/robot_localization.yaml` is the configuration for the EKF.

A graph of nodes and topics (in simulation):

![rosgraph](./docs/rosgraph.png)

### Robot

The configuration for the Jackal (startup scripts, etc.) is in /etc/ros.

The `jackal_control` package is the one in the home folder:
```bash
> rospack find jackal_control
/home/administrator/catkin_ws/src/jackal/jackal_control
```

### Sensors

The Jackal has the following sensors:
- IMU
- Odom (wheel encoder?)

Default `robot_localization` configuration found at `/opt/ros/noetic/share/jackal_control/config/robot_localization.yaml`:

```yaml
#Configuation for robot odometry EKF
#
frequency: 50

odom0: /jackal_velocity_controller/odom
odom0_config: [false, false, false, # x, y, z
               false, false, false, # roll, pitch, yaw
               true, true, true, # dx, dy, dz
               false, false, true, # droll, dpitch, dyaw
               false, false, false] # ddx, ddy, ddz
odom0_differential: false

imu0: /imu/data
imu0_config: [false, false, false,
              true, true, false,
              false, false, false,
              true, true, true,
              false, false, false]
imu0_differential: false

odom_frame: odom
base_link_frame: base_link
world_frame: odom

predict_to_current_time: true
```

### Time synchronization

The university appears to block accessing port 123 (NTP) from the university network. This causes the clock to be out of sync with the rest of the network. Below are some workarounds.

#### NTP

Run this on a host machine
```bash
sudo docker run --name=ntp --env=NTP_SERVERS="127.127.1.1" --rm --publish=123:123/udp cturra/ntp
```

Edit `/etc/systemd/timesyncd.conf` on the robot to include `NTP=<host_ip>`.

```bash
systemctl restart systemd-timesyncd # restart the time sync service
journalctl -u systemd-timesyncd -f # monitor the time sync service logs
```

##### Stratum 1 servers

[List here](https://www.advtimesync.com/docs/manual/stratum1.html)

Tested working at the time of writing:
```
ntp.student.cs.uwaterloo.ca
clock.uregina.ca
```


#### PTP

*ptp4l requires an interface that supports timestamping. Only the ethernet interface supports timestamping. Also, it didn't work on the robot when connecting the `eno1` to `wato-laptop1`'s enp4s0. So the instructions below are deprecated.*

We use `ptp4l` to synchronize the robot's clock with the master clock on the host machine.

```bash
sudo apt install linuxptp
ethtool -T <interface> # check if the interface supports hardware timestamping
ptp4l -i <interface> -m -S # -i is the interface to use, -m outputs messages to stdout, -S uses software timestamping
```
