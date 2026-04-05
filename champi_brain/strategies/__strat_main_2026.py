#!/usr/bin/env python3

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

def create_main_strategy(points_per_action: dict) -> StrategyBuilder:   
    strategy = (StrategyBuilder(points_per_action)
        # Configuration of poses
        .set_init_pose(0.4, 1.8, -90.0)    # nid jaune
        .set_home_pose(0.4, 1.8, 90)      # nid jaune
        .set_wait_to_come_home_pose(0.4, 1.4, 90.0)  # position in front of yellow home zone
        
        # Create groups
        .create_group("caisses_3")

        ## The whole strategy is always given for YELLOW team
        ## Position() coordinates are automatically transformed for BLUE team via central symmetry
        ## Elements IDs are automatically transformed via mirror_id mapping

        ######################### caisses_3 ################################
        # premier mouvement pour éviter le grenier
        .move_to(Position(0.5, 1.3, -90.0), group="caisses_3", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)
        .take_elements_sequence(Position(1.0, 0.8, 0.0), group="caisses_3")

        ######################### COME HOME ##########################################
        # movement before coming home to keep some distance from the table elements
        # .move_to(Position(0.45, 0.9, 90.0), group="come_home", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)
        # .come_home() # done automatically at the end by the planner
    )
    
    return strategy
