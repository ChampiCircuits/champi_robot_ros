#!/usr/bin/env python3

"""
Homologation strategy for 2025 competition
Simpler version focusing on banner placement and one element group
"""

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

def create_main_strategy() -> StrategyBuilder:   
    strategy = (StrategyBuilder()
        # Configuration of poses
        .set_init_pose(1.18, 0.16, 30.0)  # yellow starting zone, banner against wall
        .set_home_pose(0.3, 1.0, 0.0)      # yellow ending zone
        .set_wait_to_come_home_pose(0.3, 0.6, 0.0)  # position in front of yellow home zone
        
        # Create groups
        .create_group("banner")
        .create_group("elements_1")
        .create_group("come_home")
        
        ######################### BANNER ###############################################
        # Move forward from starting position
        .move_to(Position(1.18, 0.46, 90.0), group="banner")
        
        ######################### FIRST ELEMENT GROUP ##################################
        # Take elements from first platform
        .take_elements_sequence(Position(1.1, 0.95, 90.0), "elements_1")

        # Put elements
        .put_elements_sequence(Position(1.225, 0.1, 270.0), "elements_1")

        ######################### COME HOME ############################################
        # Move to position to look at aruco marker
        .move_to(Position(0.3, 0.6, 0.0), group="come_home", use_collision_avoidance=True)
        
        # Reset actuators before final positioning
        .custom_action("RESET_ACTUATORS", group="elements_3")
    )
    
    return strategy
