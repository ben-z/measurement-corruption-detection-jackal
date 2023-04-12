import argparse
from collections import namedtuple

def get_argument_names(parser):
    argument_names = []
    for action in parser._actions:
        # Exclude the help action (if present)
        if isinstance(action, argparse._HelpAction):
            continue
        argument_names.append(action.dest)
    return argument_names

def import_scenario_module(scenario_name):
    scenario_module_name = f"{scenario_name}_scenario"
    return getattr(getattr(__import__(f"scripts.scenarios.{scenario_module_name}"), "scenarios"), scenario_module_name)

def dict_to_namedtuple(dictionary):
    return namedtuple('Args', dictionary.keys())(*dictionary.values())

