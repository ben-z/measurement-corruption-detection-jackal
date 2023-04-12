import argparse
import time
from .scenario_base import BaseScenario, ScenarioConfig
from collections import namedtuple
from ..utils import get_argument_names, dict_to_namedtuple

def create_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'Empty scenario for testing purposes.'
    parser.add_argument('--run_duration', type=float, default=10.0)
    return parser

class EmptyScenario(BaseScenario):
    def __init__(self, args, experiment_config):
        parser = create_parser()
        my_args = {k: getattr(args, k) for k in get_argument_names(parser)}
        self.args = dict_to_namedtuple(my_args)

        self.config = ScenarioConfig(
            type='empty',
            args=my_args,
        )

    def run(self):
        print("Running empty scenario for {} seconds".format(self.args.run_duration))
        time.sleep(self.args.run_duration)

Scenario = EmptyScenario