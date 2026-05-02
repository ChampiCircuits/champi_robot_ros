#!/usr/bin/env python3

"""
Simple back-and-forth strategy for testing purposes
"""

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

num_iterations = 10


def create_main_strategy(points_per_action: dict) -> StrategyBuilder:   
    """Create a square movement strategy
    """
    strategy = StrategyBuilder(points_per_action)

    init_pose = (0.6, 0.9, -90.0)

    # Configuration of poses
    strategy.set_init_pose(init_pose[0], init_pose[1], init_pose[2])
    strategy.set_home_pose(init_pose[0], init_pose[1], init_pose[2])
    strategy.set_wait_to_come_home_pose(init_pose[0], init_pose[1], init_pose[2])

    # Add square movements
    for i in range(num_iterations):
        strategy.move_to(Position(init_pose[0]+0.5, init_pose[1], init_pose[2]), speed=1.0, accel_linear=2.0, accel_angular=15.0)
        strategy.move_to(Position(init_pose[0]+0.5, init_pose[1]+0.5, init_pose[2]), speed=1.0, accel_linear=2.0, accel_angular=15.0)
        strategy.move_to(Position(init_pose[0], init_pose[1]+0.5, init_pose[2]), speed=1.0, accel_linear=2.0, accel_angular=15.0)
        strategy.move_to(Position(init_pose[0], init_pose[1], init_pose[2]), speed=1.0, accel_linear=2.0, accel_angular=15.0)

    # Come home at the end
    strategy.come_home()

    return strategy
