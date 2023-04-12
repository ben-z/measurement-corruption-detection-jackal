from typing import NamedTuple, List
from enum import Enum
from bcontrol.corruption_generator import CorruptionGeneratorSpec

class ScenarioType(str, Enum):
    CORRUPTION = 'corruption'

class ScenarioConfig(NamedTuple):
    args: List[str]
    type: ScenarioType

class CorruptionScenarioConfig(ScenarioConfig):
    def __new__(cls, args: List[str], corruption_type: str, corruption_value: float):
        return super().__new__(cls, args, ScenarioType.CORRUPTION)
    
    spec: CorruptionGeneratorSpec

class ExperimentConfig(NamedTuple):
    experiment_name: str
    experiment_id: str
    experiments_dir: str
    experiment_dir: str
    gazebo_world: str
    gazebo_world_path: str
    real_time_factor: float
    scenario_config: ScenarioConfig
