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
        # .move_to(Position(0.4, 1.3, 180.0), group="sortie_grenier")

        .take_elements_sequence(Position(0.175, 1.2, 180.0), group="caisses_1") # OK
        .put_4_elements_sequence(Position(1.25, 1.45, 90.0), group="caisses_1") # garde_manger_4

        .take_elements_sequence(Position(1.15, 0.8, -90.0), group="caisses_3")
        .put_4_elements_sequence(Position(0.8, 0.8, 90.0), group="caisses_3") # garde_manger_3

        .take_elements_sequence(Position(1.1, 0.175, -90.0), group="caisses_4")

        .move_to(Position(1.0, 0.6, 180.0), group="see_tag_before_thermo")
        .move_thermometer("move_thermometer")

        .put_4_elements_sequence(Position(0.7, 0.1, -90.0), group="caisses_4") # garde_manger_2

        .take_elements_sequence(Position(0.175, 0.4, 180.0), group="caisses_2")
        .put_4_elements_sequence(Position(0.1, 0.8, 180.0), group="caisses_2") # garde_manger_1 # OK


        ######################### COME HOME ##########################################
        # movement before coming home to keep some distance from the table elements
        .move_to(Position(0.9, 1.3, -90.0), group="come_home", use_collision_avoidance=True)
        # .come_home() # done automatically at the end by the planner
    )
    
    return strategy
