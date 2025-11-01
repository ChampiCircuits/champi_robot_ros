#!/usr/bin/env python3

import importlib.util
import sys
import os
from pathlib import Path

def load_strategy_dsl(strategy_file_path, color, logger):
    """Load a strategy from a Python DSL file"""
    
    # Import the strategy module
    spec = importlib.util.spec_from_file_location("strategy_module", strategy_file_path)
    strategy_module = importlib.util.module_from_spec(spec)
    
    # Add module directory to PATH for imports
    strategy_dir = os.path.dirname(strategy_file_path)
    if strategy_dir not in sys.path:
        sys.path.insert(0, strategy_dir)
    
    spec.loader.exec_module(strategy_module)
    
    # Get the strategy
    if hasattr(strategy_module, 'create_main_strategy'):
        strategy_builder = strategy_module.create_main_strategy()
    else:
        raise ValueError(f"File {strategy_file_path} must contain a 'create_main_strategy()' function")
    
    # Convert to expected format
    from champi_brain.strategy_dsl import Color as DSLColor
    color_enum = DSLColor.BLUE if color == 'BLUE' else DSLColor.YELLOW
    strategy_dict = strategy_builder.to_dict(color_enum)
    
    init_pose = [
        strategy_dict['init_pose']['x'],
        strategy_dict['init_pose']['y'], 
        strategy_dict['init_pose']['theta_deg']
    ]
    
    home_pose = [
        strategy_dict['home_pose']['x'],
        strategy_dict['home_pose']['y'],
        strategy_dict['home_pose']['theta_deg']
    ]
    
    logger.info(f'<< Init pose will be {init_pose[0]} {init_pose[1]} {init_pose[2]}°!')
    logger.info(f'<< Home pose will be {home_pose[0]} {home_pose[1]} {home_pose[2]}°!')
    
    actions = strategy_dict['actions']
    
    for (i, action) in enumerate(actions):
        logger.info(f'Action {i}: {action}')
    
    return actions, init_pose, home_pose

def load_strategy(file_path, color, logger):
    """Main entry point to load a strategy"""
    
    file_ext = Path(file_path).suffix.lower()
    
    if file_ext == '.py':
        # Python DSL format
        logger.info(f"Loading Python DSL strategy: {file_path}")
        return load_strategy_dsl(file_path, color, logger)
    
    else:
        raise ValueError(f"Unsupported file format: {file_ext}. Use .py only")
