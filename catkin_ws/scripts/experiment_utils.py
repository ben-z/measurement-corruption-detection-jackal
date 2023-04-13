from typing import NamedTuple, List, Optional, Any, Dict
from enum import Enum
from pathlib import Path
import sys
from .scenarios.scenario_base import ScenarioConfig

# Add catkin_ws/src to PYTHONPATH
catkin_ws_src_path = Path(__file__).parent.parent / 'src'
if str(catkin_ws_src_path) not in sys.path:
    sys.path.append(str(catkin_ws_src_path))

from bcontrol.src.corruption_generator import CorruptionGeneratorSpec

class ExperimentConfig(NamedTuple):
    experiment_name: str
    experiment_id: str
    experiments_dir: str
    experiment_dir: str
    gazebo_world: str
    gazebo_world_path: str
    real_time_factor: float
    planner_profile: str
    scenario_config: Optional[ScenarioConfig]
