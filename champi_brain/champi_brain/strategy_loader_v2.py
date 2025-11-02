#!/usr/bin/env python3

import importlib.util
import sys
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def load_points_per_action():
    """Load points per action from a YAML file"""

    points_yaml_file = get_package_share_directory('champi_brain') + '/strategies/points_per_action.yaml'
    with open(points_yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    return data.get('points_per_action', {})

def load_strategy_dsl(strategy_file_path, color, logger):
    """Load a strategy from a Python DSL file - returns typed objects"""
    
    # Import the strategy module
    spec = importlib.util.spec_from_file_location("strategy_module", strategy_file_path)
    strategy_module = importlib.util.module_from_spec(spec)

    # Load the points per action configuration
    points_per_action = load_points_per_action()
    logger.info(f'Loaded points per action configuration: {points_per_action}')
    
    # Add module directory to PATH for imports
    strategy_dir = os.path.dirname(strategy_file_path)
    if strategy_dir not in sys.path:
        sys.path.insert(0, strategy_dir)
    
    spec.loader.exec_module(strategy_module)
    
    # Get the strategy
    if hasattr(strategy_module, 'create_main_strategy'):
        strategy_builder = strategy_module.create_main_strategy(points_per_action)
    else:
        raise ValueError(f"File {strategy_file_path} must contain a 'create_main_strategy()' function")
    
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
        logger.info(f'Action {i}: {action}')
    
    return actions, init_pose, home_pose, wait_to_come_home_pose

def load_strategy(file_path, color, logger):
    """Main entry point to load a strategy"""
    
    file_ext = Path(file_path).suffix.lower()
    
    if file_ext == '.py':
        # Python DSL format
        logger.info(f"Loading Python DSL strategy: {file_path}")
        return load_strategy_dsl(file_path, color, logger)
    
    else:
        raise ValueError(f"Unsupported file format: {file_ext}. Use .py only")
