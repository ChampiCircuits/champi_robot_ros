#!/usr/bin/env python3

import importlib.util
import sys, os, yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from champi_brain.world_state.symmetry import init_symmetry_mapper

def initialize_symmetry_mapper(logger, initial_world_state_path: str):
    """Initialize the symmetry mapper from the world state config file."""

    with open(initial_world_state_path, 'r') as f:
        data = yaml.safe_load(f)
    
    raw_elements = data.get('elements', [])
    raw_zones = data.get('zones', [])
    
    init_symmetry_mapper(raw_elements, raw_zones)
    logger.info('✅ Symmetry mapper initialized from initial_world_state_2025.yaml')


def load_points_per_action():
    """Load points per action from a YAML file"""

    points_yaml_file = get_package_share_directory('champi_brain') + '/strategies/points_per_action.yaml'
    with open(points_yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    return data.get('points_per_action', {})

def load_time_per_action():
    """Load time per action from a YAML file"""

    time_yaml_file = get_package_share_directory('champi_brain') + '/strategies/time_per_action.yaml'
    with open(time_yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    return data.get('time_per_action', {})

def load_strategy_dsl(strategy_file_path, color, logger, initial_world_state_path):
    """Load a strategy from a Python DSL file - returns typed objects"""
    
    # Initialize symmetry mapper if not already done
    initialize_symmetry_mapper(logger, initial_world_state_path)
    
    # Import the strategy module
    spec = importlib.util.spec_from_file_location("strategy_module", strategy_file_path)
    if spec is None:
        raise ValueError(f"Could not load spec from {strategy_file_path}")
    strategy_module = importlib.util.module_from_spec(spec)

    # Load the points per action configuration
    points_per_action = load_points_per_action()
    logger.info(f'Loaded points per action configuration: {points_per_action}')

    # Load the time per action configuration
    time_per_action = load_time_per_action()
    logger.info(f'Loaded time per action configuration: {time_per_action}')

    # Add module directory to PATH for imports
    strategy_dir = os.path.dirname(strategy_file_path)
    if strategy_dir not in sys.path:
        sys.path.insert(0, strategy_dir)
    
    if spec.loader is None:
        raise ValueError(f"Spec loader is None for {strategy_file_path}")
    spec.loader.exec_module(strategy_module)
    
    # Get the strategy
    strategy_builder = strategy_module.create_main_strategy(points_per_action)

    # Get typed objects instead of dicts
    from champi_brain.strategy_dsl import Color as DSLColor
    color_enum = DSLColor.BLUE if color == 'BLUE' else DSLColor.YELLOW
    
    # Get transformed typed objects
    actions = strategy_builder.get_transformed_actions(color_enum)
    init_pose_obj = strategy_builder.get_init_pose(color_enum)
    home_pose_obj = strategy_builder.get_home_pose(color_enum)
    wait_to_come_home_pose_obj = strategy_builder.get_wait_to_come_home_pose(color_enum)
    
    # Convert poses to list format for compatibility
    init_pose = [init_pose_obj.x, init_pose_obj.y, init_pose_obj.theta_deg]
    home_pose = [home_pose_obj.x, home_pose_obj.y, home_pose_obj.theta_deg]
    wait_to_come_home_pose = [wait_to_come_home_pose_obj.x, wait_to_come_home_pose_obj.y, wait_to_come_home_pose_obj.theta_deg]
    
    logger.info(f'<< Init pose will be {init_pose[0]} {init_pose[1]} {init_pose[2]}°!')
    logger.info(f'<< Home pose will be {home_pose[0]} {home_pose[1]} {home_pose[2]}°!')
    logger.info(f'<< Wait to come home pose will be {wait_to_come_home_pose[0]} {wait_to_come_home_pose[1]} {wait_to_come_home_pose[2]}°!')
    
    for (i, action) in enumerate(actions):
        logger.debug(f'Action {i}: {action}')
    
    return actions, init_pose, home_pose, wait_to_come_home_pose, time_per_action

def load_strategy(file_path, color, logger, initial_world_state_path):
    """Main entry point to load a strategy
    
    Args:
        file_path: Path to the strategy file
        color: Team color ('YELLOW' or 'BLUE')
        logger: Logger instance
        initial_world_state_path: Path to the initial world state YAML file
    
    Returns:
        tuple: (actions, init_pose, home_pose, wait_to_come_home_pose, time_per_action)
    """
    
    file_ext = Path(file_path).suffix.lower()
    
    if file_ext == '.py':
        # Python DSL format
        logger.info(f"Loading Python DSL strategy: {file_path}")
        return load_strategy_dsl(file_path, color, logger, initial_world_state_path)
    else:
        raise ValueError(f"Unsupported file format: {file_ext}. Use .py only")
