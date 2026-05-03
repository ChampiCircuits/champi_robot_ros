#!/usr/bin/env python3

"""
Homologation strategy for 2026 competition
Simpler version focusing on one element group
"""

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

# TODO use directly positions from world_state

def create_main_strategy(points_per_action: dict) -> StrategyBuilder:
    strategy = (StrategyBuilder(points_per_action)
        # Configuration of poses
        .set_init_pose(0.4, 1.8, -90.0) # nid jaune
        .set_home_pose(0.4, 1.8, -90.0) # nid jaune
        .set_wait_to_come_home_pose(0.4, 1.4, 90.0)  # position in front of yellow home zone

        # Create groups
        .create_group("caisses_1")
        .create_group("caisses_4")

        ## The whole strategy is always given for YELLOW team
        ## Position() coordinates are automatically transformed for BLUE team via central symmetry
        ## Elements IDs (when used) are automatically transformed via mirror_id mapping

        .take_elements_sequence(Position(0.175, 1.2, 180.0), group="caisses_1") # OK
        .put_4_elements_sequence(Position(0.7, 0.1, -90.0), group="caisses_1") # garde_manger_2

        .take_elements_sequence(Position(1.1, 0.175, -90.0), group="caisses_4")
        .put_4_elements_sequence(Position(1.5, 0.1, -90.0), group="caisses_4") # garde_manger_5


        ######################### COME HOME ##########################################
        # movement before coming home to keep some distance from the table elements
        .move_to(Position(0.4, 1.3, -90.0), group="come_home", use_collision_avoidance=True)
        .come_home() # done automatically at the end by the planner, but manually here for homologation
        )

    return strategy
