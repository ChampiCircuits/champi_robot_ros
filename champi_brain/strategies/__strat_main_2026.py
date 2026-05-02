#!/usr/bin/env python3

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color
from champi_brain.actuator_commands import ActuatorCommand


# TODO use directly positions from world_state

def create_main_strategy(points_per_action: dict) -> StrategyBuilder:   
    strategy = (StrategyBuilder(points_per_action)
        # Configuration of poses
        .set_init_pose(0.4, 1.8, -90.0) # nid jaune
        .set_home_pose(0.4, 1.8, -90.0) # nid jaune
        .set_wait_to_come_home_pose(0.4, 1.4, 90.0)  # position in front of yellow home zone
        
        # Create groups
        .create_group("caisses_3")
        .create_group("caisses_4")
        .create_group("caisses_2")
        .create_group("caisses_1")
        .create_group("move_thermometer")

        ## The whole strategy is always given for YELLOW team
        ## Position() coordinates are automatically transformed for BLUE team via central symmetry
        ## Elements IDs (when used) are automatically transformed via mirror_id mapping

        # premier mouvement pour éviter le grenier
        # .move_to(Position(0.4, 0.8, -90.0), group="caisses_3", speed=1.0, accel_linear=1.0)

        .take_elements_sequence(Position(1.15, 0.8, 0.0), group="caisses_3")
        
        # .move_to(Position(1.5, 0.175, 180.0), group="caisses_4", speed=1.0, accel_linear=1.0)
        # .take_elements_sequence(Position(1.1, 0.175, 180.0), group="caisses_4")

        .move_thermometer("move_thermometer")

        # .custom_action(ActuatorCommand.OPEN_EXIT_RAMP, group="caisses_3")
        # .put_4_elements_sequence(Position(1.5, 0.1, 180.0), group="caisses_3") # garde_manger_5
        # .put_4_elements_sequence(Position(0.7, 0.1, 180.0), group="caisses_4") # garde_manger_2
        #
        # .take_elements_sequence(Position(0.175, 0.4, 90.0), group="caisses_2")
        # .put_4_elements_sequence(Position(0.1, 0.8, 90.0), group="caisses_2") # garde_manger_1
        #
        # .take_elements_sequence(Position(0.175, 1.2, 90.0), group="caisses_1")
        #
        # .put_2_elements_sequence(Position(1.25, 1.45, 180.0), group="caisses_1") # garde_manger_4
        # .move_to(Position(0.4, 1.45, 90.0), group="caisses_1", speed=1.0, accel_linear=1.0)
        #
        # .put_last_2_elements_in_nest_sequence(Position(0.4, 1.8, 90.0), group="caisses_1")  # nid_jaune

        ######################### COME HOME ##########################################
        # movement before coming home to keep some distance from the table elements
        # .move_to(Position(0.45, 0.9, 90.0), group="come_home", use_collision_avoidance=True, speed=1.0, accel_linear=1.0, accel_angular=15.0)
        # .come_home() # done automatically at the end by the planner
    )
    
    return strategy
