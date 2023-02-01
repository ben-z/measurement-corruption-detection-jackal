# Jackal2 Configuration

`ben_jackal2.vsk`: VICON configuration for the Jackal2 object. This contains 3 markers around the body and 3 markers surrounding the IMU unit.

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

Start the Gazebo simulator:

```bash
# Environmental variable configuration reference: https://github.com/jackal/jackal/commit/75c68523945b0edf504c64b8b188260292bb3472
export JACKAL_LASER_3D=1 # Enable 3D laser
DISPLAY=:1.0 roslaunch jackal_gazebo jackal_world.launch world_name:=/workspace/gazebo-worlds/empty.world
DISPLAY=:1.0 roslaunch jackal_viz view_robot.launch
```

Development:

```bash
cd /workspace/catkin_ws
catkin build
source devel/setup.bash
```
