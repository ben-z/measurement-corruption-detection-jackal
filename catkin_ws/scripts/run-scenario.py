#!/usr/bin/env python3

import argparse
import rospy
import rospkg
import roslaunch
from datetime import datetime
from slugify import slugify
import os
from pathlib import Path
import time
from xml.etree import ElementTree
import yaml
import sys


GAZEB_WORLDS_DIR = Path('/workspace/gazebo-worlds')

def ros_prerun(nodes):
    prerun_launch = roslaunch.parent.ROSLaunchParent('prerun', [], is_core=True)
    try:
        prerun_launch.start()

        rospy.init_node('scenario_runner_prerun_node')

        rospy.loginfo("Starting nodes: %s", [n.name or f"{n.package}/{n.type}" for n in nodes])

        node_launcher = roslaunch.scriptapi.ROSLaunch()
        node_launcher.start()
        processes = [node_launcher.launch(node) for node in nodes]
        while any([p.is_alive() for p in processes]):
            rospy.loginfo_throttle(1, f"Waiting for {[p.name for p in processes if p.is_alive]} to finish...")
            node_launcher.spin_once()
        node_launcher.stop()
    
        errored_processes = [p for p in processes if p.exit_code != 0]
        if errored_processes:
            error_msg = f"Prerun failed: {[f'{p.name} (code={p.exit_code})' for p in errored_processes]}"
            rospy.logfatal(error_msg)
            raise Exception(error_msg)
    finally:
        prerun_launch.shutdown()

def extract_gazebo_real_time_factor(gazebo_world_path):
    gazebo_physics_settings = ElementTree.parse(gazebo_world_path).getroot().find('.//physics')
    assert gazebo_physics_settings is not None, f"Physics settings not found in {gazebo_world_path}"
    gazebo_real_time_factor = gazebo_physics_settings.find('real_time_factor')
    gazebo_real_time_update_rate = gazebo_physics_settings.find('real_time_update_rate')
    gazebo_max_step_size = gazebo_physics_settings.find('max_step_size')
    if gazebo_real_time_update_rate is not None and gazebo_max_step_size is not None:
        assert gazebo_real_time_update_rate.text is not None, f"Impossible: real_time_update_rate.text is None but real_time_update_rate exists"
        assert gazebo_max_step_size.text is not None, f"Impossible: max_step_size.text is None but max_step_size exists"
        real_time_factor_bound = float(gazebo_real_time_update_rate.text) * float(gazebo_max_step_size.text)
    else: 
        real_time_factor_bound = 1.0
    
    if gazebo_real_time_factor is not None:
        assert gazebo_real_time_factor.text is not None, f"Impossible: real_time_factor.text is None but real_time_factor exists"
        real_time_factor = min(float(gazebo_real_time_factor.text), real_time_factor_bound)
    else:
        real_time_factor = real_time_factor_bound
    
    return real_time_factor

def sleep_simtime(duration, real_time_factor):
    """
    Sleeps for the given duration in simulation time.

    Arguments:
        duration {float} -- duration in seconds
        real_time_factor {float} -- Simulation time / real time
    """
    rospy.sleep(duration / real_time_factor)

def main(args, unknown_args):
    # Create experiment directory
    experiments_dir = Path(args.experiments_dir)
    experiment_full_name = f"{args.experiment_id}-{args.experiment_name}"
    experiment_dir = experiments_dir / experiment_full_name
    print(f"Creating experiment directory: {experiment_dir}")
    experiment_dir.mkdir(parents=True)

    # Process argumetns
    gazebo_world_path = GAZEB_WORLDS_DIR / \
        Path(args.gazebo_world).with_suffix('.world')
    if not gazebo_world_path.exists():
        raise Exception(f"Gazebo world not found: {gazebo_world_path}")

    real_time_factor = extract_gazebo_real_time_factor(gazebo_world_path)
    print(f"Using Gazebo real time factor: {real_time_factor} (from {gazebo_world_path})")

    # Write configuration file to the experiment directory
    with open(experiment_dir / 'config.yaml', 'w') as f:
        yaml.dump({
            **vars(args),
            'gazebo_world_path': str(gazebo_world_path),
            'real_time_factor': real_time_factor,
        }, f)

    # Set environment variables
    os.environ['ROS_LOG_DIR'] = str(experiment_dir / 'ros_logs')
    # disable the built-in EKF in Jackal
    os.environ['ENABLE_EKF'] = 'false'

    # Find packages
    rospack = rospkg.RosPack()
    bcontrol_path = rospack.get_path('bcontrol')

    # Generate dynamic launch files 
    detector_pipeline_launch_file = Path(bcontrol_path) / 'launch/detector_pipeline.generated.launch'
    detector_pipeline_generator_node = roslaunch.core.Node(
        'bcontrol', 'generate_detector_pipeline_launch_file.py',
        name='detector_pipeline_generator',
        args=f"{bcontrol_path}/config/bdetect.yaml {detector_pipeline_launch_file}"
    )
    ros_prerun([detector_pipeline_generator_node])
    assert detector_pipeline_launch_file.exists(), f"Detector pipeline launch file not found: {detector_pipeline_launch_file}"

    # Launch the base stack
    base_launch = roslaunch.parent.ROSLaunchParent(
        "scenario_runner",
        [
            (
                bcontrol_path + "/launch/sim.launch",
                [
                    "enable_foxglove:=false",
                    f"gazebo_world:={gazebo_world_path}",
                ]
            ),
            (
                bcontrol_path + "/launch/stack.launch",
                [
                    "enable_detector:=false",
                ]
            ),
        ],
        sigint_timeout=2,
        is_core=True,
    )
    base_launch.start()

    try:
        base_launch.spin()
    finally:
        base_launch.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--experiment_name', type=str, default="experiment")
    parser.add_argument('--experiment_id', type=str,
                        default=slugify(datetime.now().isoformat()))
    parser.add_argument('--experiments_dir', type=str, default='/experiments')
    parser.add_argument('--gazebo_world', type=str, default='empty-rate_200')

    args, unknown_args = parser.parse_known_args()
    main(args, unknown_args)
