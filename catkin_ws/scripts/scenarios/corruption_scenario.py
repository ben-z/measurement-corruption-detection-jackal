import argparse
import time
from .scenario_base import BaseScenario, ScenarioConfig
from collections import namedtuple
from ..utils import get_argument_names, dict_to_namedtuple
from ..experiment_utils import ExperimentConfig
import roslaunch
from pathlib import Path
import rospy
import rospkg

import bcontrol.src.corruption_generator as cg


def create_parser(create_parser_fn=None):
    if create_parser_fn is None:
        create_parser_fn = argparse.ArgumentParser
    corruption_generator_parser = cg.create_parser(add_help=False)
    parser = create_parser_fn(parents=[corruption_generator_parser])
    parser.description = 'Corruption scenario'
    parser.add_argument('--corruption_duration', type=float, default=10.0, help='Duration of the corruption in seconds.')
    parser.add_argument('--detector_solve_hz', type=float, help='Frequency at which the detector should solve.')
    return parser


class CorruptionScenario(BaseScenario):
    def __init__(self, args, experiment_config: ExperimentConfig):
        parser = create_parser()
        my_args = {k: getattr(args, k) for k in get_argument_names(parser)}
        self.args = dict_to_namedtuple(my_args)
        self.experiment_config = experiment_config

        self.config = ScenarioConfig(
            type='corruption',
            args=my_args,
        )

        # Start recording
        rosbag_path = Path(self.experiment_config.experiment_dir) / 'recording.bag'
        self.rosbag_node = roslaunch.core.Node(
            'rosbag', 'record',
            name='rosbag_recorder',
            args=f"-j -a -O {rosbag_path}",
        )

        rospack = rospkg.RosPack()
        bcontrol_path = rospack.get_path('bcontrol')

        self.detector_launch = roslaunch.parent.ROSLaunchParent(
            self.experiment_config.experiment_name,
            [
                (
                    bcontrol_path + "/launch/detector.launch",
                    [
                        f"detector_solve_hz:={self.args.detector_solve_hz or ''}",
                    ],
                )
            ],
            sigint_timeout=2,
        )

        self.corruption_spec = cg.create_spec(self.args)
        self.corruption_generator_node = cg.CorruptionGeneratorNode(spec=self.corruption_spec)

    def run(self):
        self.node_launcher = roslaunch.scriptapi.ROSLaunch()
        self.node_launcher.start()

        rospy.loginfo("Starting rosbag recorder...")
        self.node_launcher.launch(self.rosbag_node)

        rospy.loginfo("Waiting for steady state...")
        rospy.sleep(15)

        # Start detector
        rospy.loginfo("Starting detector...")
        self.detector_launch.start()
        rospy.sleep(10) # wait for the detector to collect data

        rospy.loginfo("Initializing corruption generator...")
        self.corruption_generator_node.init()

        rospy.sleep(self.args.corruption_duration + self.corruption_spec.corruption_start_sec)

        rospy.loginfo("Done performing attack. Shutting down the corruption generator...")
        self.corruption_generator_node.shutdown()

        rospy.loginfo("Waiting for recovery...")
        rospy.sleep(15)

    def cleanup(self):
        node_launcher = getattr(self, 'node_launcher', None)
        if node_launcher is not None:
            node_launcher.stop()


Scenario = CorruptionScenario
