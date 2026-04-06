#!/usr/bin/env python3

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

def create_main_strategy(points_per_action: dict) -> StrategyBuilder:   
    strategy = (StrategyBuilder(points_per_action)
        # Configuration of poses
        .set_init_pose(0.4, 1.8, -90.0)    # nid jaune
        .set_home_pose(0.4, 1.8, -90.0)      # nid jaune
        .set_wait_to_come_home_pose(0.4, 1.4, 90.0)  # position in front of yellow home zone
        
        # Create groups
        .create_group("caisses_3")

        ## The whole strategy is always given for YELLOW team
        ## Position() coordinates are automatically transformed for BLUE team via central symmetry
        ## Elements IDs are automatically transformed via mirror_id mapping

        # premier mouvement pour éviter le grenier
        .move_to(Position(0.5, 1.3, -90.0), group="caisses_3",
                 speed=1.0, accel_linear=1.0,
                 linear_tolerance=0.1, angular_tolerance=0.3, end_speed=0.7)

        .take_elements_sequence(Position(1.15, 0.8, 0.0), group="caisses_3") # TODO use directly positions from world_state
        .take_elements_sequence(Position(1.1, 0.175, 180.0), group="caisses_4")

        .move_thermometer("move_thermometer")

        .put_elements_sequence(Position(1.5, 0.1, 180.0), group="caisses_3") # garde_manger_5
        .put_elements_sequence(Position(0.7, 0.1, 180.0), group="caisses_4") # garde_manger_2

        .take_elements_sequence(Position(0.175, 0.4, 90.0), group="caisses_2")
        .put_elements_sequence(Position(0.1, 0.8, 90.0), group="caisses_2") # garde_manger_1

        .take_elements_sequence(Position(0.175, 1.2, 90.0), group="caisses_1")
        .put_elements_sequence(Position(0.4, 1.8, -90.0), group="caisses_1") # nid_jaune

        ######################### COME HOME ##########################################
        # movement before coming home to keep some distance from the table elements
        # .move_to(Position(0.45, 0.9, 90.0), group="come_home", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)
        # .come_home() # done automatically at the end by the planner
    )
    
    return strategy
