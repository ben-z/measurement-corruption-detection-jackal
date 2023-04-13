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
from .experiment_utils import ExperimentConfig
from .utils import import_scenario_module, get_argument_names, dict_to_namedtuple
from multiprocessing import Process

def named_tuple_representer(self, data):
    if hasattr(data, '_asdict'):
        return self.represent_dict(data._asdict())
    return self.represent_list(data)

yaml.SafeDumper.add_multi_representer(tuple, named_tuple_representer)

GAZEB_WORLDS_DIR = Path('/workspace/gazebo-worlds')

def ros_prerun(nodes):
    prerun_launch = roslaunch.parent.ROSLaunchParent('prerun', [], sigint_timeout=2, is_core=True)
    try:
        prerun_launch.start()

        rospy.init_node('run_experiment_prerun_node')
        rospy.loginfo(f"Starting nodes: {[n.name or f'{n.package}/{n.type}' for n in nodes]}")

        node_launcher = roslaunch.scriptapi.ROSLaunch()
        node_launcher.start()
        processes = [node_launcher.launch(node) for node in nodes]
        while any([p.is_alive() for p in processes]):
            rospy.loginfo_throttle(1.0, f"Waiting for {[p.name for p in processes if p.is_alive]} to finish...")
            node_launcher.spin_once()
        node_launcher.stop()
    
        errored_processes = [p for p in processes if p.exit_code != 0]
        if errored_processes:
            error_msg = f"Prerun failed: {[f'{p.name} (code={p.exit_code})' for p in errored_processes]}"
            rospy.logfatal(error_msg, file=sys.stderr)
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

def main(experiment_args, downstream_args):
    # Create experiment directory
    experiments_dir = Path(experiment_args.experiments_dir)
    experiment_full_name = f"{experiment_args.experiment_id}-{experiment_args.experiment_name}"
    experiment_dir = experiments_dir / experiment_full_name
    print(f"Creating experiment directory: {experiment_dir}")
    experiment_dir.mkdir(parents=True)

    # Process argumetns
    gazebo_world_path = GAZEB_WORLDS_DIR / \
        Path(experiment_args.gazebo_world).with_suffix('.world')
    if not gazebo_world_path.exists():
        raise Exception(f"Gazebo world not found: {gazebo_world_path}")

    real_time_factor = extract_gazebo_real_time_factor(gazebo_world_path)
    print(f"Using Gazebo real time factor: {real_time_factor} (from {gazebo_world_path})")

    experiment_config = ExperimentConfig(
        experiment_name=experiment_args.experiment_name,
        experiment_id=experiment_args.experiment_id,
        experiments_dir=experiment_args.experiments_dir,
        experiment_dir=str(experiment_dir),
        gazebo_world=experiment_args.gazebo_world,
        gazebo_world_path=str(gazebo_world_path),
        real_time_factor=real_time_factor,
        planner_profile=experiment_args.planner_profile,
        scenario_config=None,
    )

    # Initialize the scenario
    scenario_module = import_scenario_module(experiment_args.scenario)
    scenario = scenario_module.Scenario(downstream_args, experiment_config)
    print(f"Initialized scenario {scenario.__class__.__name__}")
    experiment_config = experiment_config._replace(scenario_config=scenario.config)

    # Write configuration file to the experiment directory
    with open(experiment_dir / 'config.yaml', 'w') as f:
        yaml.safe_dump(experiment_config._asdict(), f)

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
    
    # Run prerun in a separate process to not pollute the ROS global variables
    p = Process(target=ros_prerun, args=([detector_pipeline_generator_node],))
    p.start()
    p.join()
    assert detector_pipeline_launch_file.exists(), f"Detector pipeline launch file not found: {detector_pipeline_launch_file}"

    # Launch the base stack
    base_launch = roslaunch.parent.ROSLaunchParent(
        experiment_config.experiment_name,
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
                    f"planner_profile:={experiment_config.planner_profile}",
                ]
            ),
        ],
        sigint_timeout=2,
        is_core=True,
    )
    
    try:
        base_launch.start()
        rospy.init_node('experiment_runner', log_level=rospy.INFO)
        rospy.loginfo("Starting scenario")
        scenario.run()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Scenario interrupted. Exiting...")
    finally:
        scenario.cleanup()
        base_launch.shutdown()

if __name__ == '__main__':
    scenario_files = list((Path(__file__).parent / 'scenarios').glob('*_scenario.py'))
    scenarios = [f.stem.replace('_scenario', '') for f in scenario_files]

    parser = argparse.ArgumentParser()
    parser.add_argument('--experiment_name', type=str, default="experiment")
    parser.add_argument('--experiment_id', type=str,
                        default=slugify(datetime.now().isoformat()))
    parser.add_argument('--experiments_dir', type=str, default='/experiments')
    parser.add_argument('--gazebo_world', type=str, default='empty-rate_200')
    parser.add_argument('--planner_profile', type=str, default='')

    # Add scenario argument parsers
    scenario_parser = parser.add_subparsers(title='scenario', dest='scenario', description='Scenario to run', required=True)
    for scenario in scenarios:
        scenario_module = import_scenario_module(scenario)
        if getattr(scenario_module, 'SKIP', False):
            continue
        scenario_module.create_parser(lambda **kwargs: scenario_parser.add_parser(scenario, **kwargs))

    args = parser.parse_args()
    experiment_args = {k: getattr(args, k) for k in get_argument_names(parser)}
    downstream_args = {k: v for k, v in vars(args).items() if k not in experiment_args}

    main(dict_to_namedtuple(experiment_args), dict_to_namedtuple(downstream_args))
