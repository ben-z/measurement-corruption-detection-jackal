from typing import NamedTuple, List, Any, Dict

class ScenarioConfig(NamedTuple):
    type: str
    args: Dict[str, Any]

class BaseScenario:
    config: ScenarioConfig

    def __init__(self, argv, experiment_config):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError
    
    def cleanup(self):
        pass