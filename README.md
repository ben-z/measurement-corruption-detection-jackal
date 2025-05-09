# measurement-corruption-detection-jackal

This repository contains the deployment code for the thesis "Optimal Sensor Protection and Measurement Corruption Detection in Safety-Critical Systems" to a Clearpath Jackal robot.

There are a few components in this project. They include:
1. `robot.launch`: Launches the basic services required for the robot to operate. This includes positioning services, transform publishers, etc.
1. `sim.launch`: Same as `robot.launch`, but for simulation.
1. `stack.launch`: Launches our custom stack. This includes the controller, the planner, the localiser, the detector, etc.
1. `visualization.launch`: Launches visualization nodes. Requires a display.

We can run the stack in the following configurations in the real world:
| `robot.launch` | `stack.launch` | `visualization.launch` | Comments |
| --- | --- | --- | --- |
| Robot | Robot | Workstation | The ideal setup. Pretty much every thing runs on the robot, we use the workstation for monitoring only. |
| Workstation | Workstation | Workstation | This is used for debugging or when the robot has insufficient resources. |


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
# -o adds a prefix to the file name
rosbag record -j -a -o <recording-prefix>
```

## Development on the robot

Visualizations:

```bash
ros_connect_to_jackal2 # shortcut defined in ~/.commonrc (version controlled in personal dotfiles repo)
cd ~/Projects/research-jackal/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source ./devel/setup.zsh
DISPLAY=:0 roslaunch bcontrol visualization.launch
```

Running the stack on the robot:

```bash
ros-stop # Stop the default stack. We have our own end-to-end stack.
docker compose up -d --build robot
docker compose exec robot bash
devsetup
rosdep_install_all
catkin build
source ./devel/setup.bash
# Generate a launch file for the extractors
rosrun bcontrol generate_detector_pipeline_launch_file.py $(rospack find bcontrol)/config/bdetect.yaml $(rospack find bcontrol)/launch/detector_pipeline.generated.launch
ENABLE_EKF=false roslaunch bcontrol robot.launch
# Launch the stack without the detector
roslaunch bcontrol stack.launch enable_detector:=false

# Launch the detector (Choose one of the following)
roslaunch bcontrol detector.launch # or
roslaunch bcontrol detector.launch open_loop:=true # or
roslaunch bcontrol detector.launch debug:=true # for interactive debugging, or
roslaunch bcontrol detector.launch launch_solver_server:=false # to disable the solver server (launch it separately)

# Launch the solver server (Choose one of the following)
rosrun bcontrol solver_server.py # or
rosrun bcontrol solver_server_debug.py # for interactive debugging
```

Note that by default, only 1 interactive debugging node can be run at a time. To run multiple interactive debugging nodes, we need to change the hard-coded port number in the corresponding debug files.

## Simulation

Start the Docker container:

```bash
./generate-dot-env.sh
docker compose up -d --build
docker compose exec sim bash
```

All commands from now on should be run inside the container.

cd into the catkin workspace:

```bash
devsetup # shortcut for cd-ing into the catkin_ws dir and sourcing setup.bash (defined in /etc/local.bashrc)
```

Install dependencies:

```bash
rosdep_install_all # shortcut (defined in /etc/local.bashrc)
```

Build the workspace:

```bash
catkin build
```

Start the simulation and the stack:

```bash
# Launch the simulation
VGL_DISPLAY=egl0 DISPLAY=:1.0 ENABLE_EKF=false vglrun roslaunch --sigint-timeout=2 bcontrol sim.launch # with GPU, or
DISPLAY=:1.0 ENABLE_EKF=false roslaunch --sigint-timeout=2 bcontrol sim.launch # without GPU
# Launch visualizations
VGL_DISPLAY=egl0 DISPLAY=:1.0 vglrun roslaunch bcontrol visualization.launch # with GPU, or
DISPLAY=:1.0 roslaunch bcontrol visualization.launch # without GPU
# Generate a launch file for the extractors
rosrun bcontrol generate_detector_pipeline_launch_file.py $(rospack find bcontrol)/config/bdetect.yaml $(rospack find bcontrol)/launch/detector_pipeline.generated.launch
# Launch the stack without the detector
roslaunch bcontrol stack.launch enable_detector:=false

# Launch the detector (Choose one of the following)
roslaunch bcontrol detector.launch # or
roslaunch bcontrol detector.launch open_loop:=true # or
roslaunch bcontrol detector.launch debug:=true # for interactive debugging, or
roslaunch bcontrol detector.launch launch_solver_server:=false # to disable the solver server (launch it separately)

# Launch the solver server (Choose one of the following)
rosrun bcontrol solver_server.py # or
rosrun bcontrol solver_server_debug.py # for interactive debugging
```

Note that by default, only 1 interactive debugging node can be run at a time. To run multiple interactive debugging nodes, we need to change the hard-coded port number in the corresponding debug files.

`VGL_DISPLAY=egl0` is used to tell VirtualGL to use the first EGL device. Each EGL device corresponds to a `/dev/dri/card*` device.

Useful VGL commands:

```bash
sudo /opt/VirtualGL/bin/eglinfo -e # list available EGL devices
```

## Related Links
- Simulation code for the same thesis: https://github.com/ben-z/measurement-corruption-detection-sim
