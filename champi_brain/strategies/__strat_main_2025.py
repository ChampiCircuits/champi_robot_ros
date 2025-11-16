#!/usr/bin/env python3

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

def create_main_strategy(points_per_action: dict) -> StrategyBuilder:   
    strategy = (StrategyBuilder(points_per_action)
        # Configuration of poses
        .set_init_pose(1.18, 0.16, 30.0)  # yellow starting zone
        .set_home_pose(0.3, 1.8, 0.0)      # yellow ending zone
        .set_wait_to_come_home_pose(0.3, 1.4, 0.0)  # position in front of yellow home zone
        
        # Create groups
        .create_group("banner")
        .create_group("elements_1")
        .create_group("elements_2")
        .create_group("elements_3")
        .create_group("come_home")

        ## The whole strategy is always given for YELLOW team
        ## Position() coordinates are automatically transformed for BLUE team via central symmetry
        
        ######################### BANNER ###############################################
        .put_banner("banner")
        .move_to(Position(1.18, 0.46, 90.0), group="banner", use_collision_avoidance=False, end_speed=0.2)

        ######################### FIRST ELEMENT GROUP ################################
        # TODO on devrait aussi pouvoir donner un genre de côté par lequel on approche l'élément
        .get_ready("elements_1")
        .take_elements_sequence("elements_1", group="elements_1")
        # Intermediate movement
        .move_to(Position(1.225, 0.6, 180.0), group="elements_1", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=12.0)
        .put_elements_sequence(Position(1.225, 0.1, 270.0), "elements_1")

        ######################### SECOND ELEMENT GROUP ###############################
        .get_ready("elements_2")
        .move_to(Position(0.9, 0.75, 90.0), group="elements_2", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)
        .take_elements_sequence("elements_2", group="elements_2")
        .put_elements_sequence(Position(1.225, 0.28, 270.0), "elements_2")

        ######################### THIRD ELEMENT GROUP ################################
        .get_ready("elements_3")
        .move_to(Position(0.9, 0.6, 90.0), group="elements_3", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)
        .take_elements_sequence("elements_3", group="elements_3")
        .put_elements_sequence(Position(0.6, 0.1, 270.0), "elements_3")
        
        ######################### COME HOME ##########################################
        .move_to(Position(0.45, 0.9, 90.0), group="come_home", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)

        # .come_home() # done automatically at the end by the planner
    )
    
    return strategy
