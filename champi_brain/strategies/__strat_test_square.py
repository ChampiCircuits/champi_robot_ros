#!/usr/bin/env python3

"""
Simple back-and-forth strategy for testing purposes
"""

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

num_iterations = 10


def create_main_strategy() -> StrategyBuilder:   
    """Create a square movement strategy
    """
    strategy = StrategyBuilder()

    # Configuration of poses
    strategy.set_init_pose(1.0, 1.0, 0.0)
    strategy.set_home_pose(1.0, 1.0, 0.0)

    # Add square movements
    for i in range(num_iterations):
        strategy.move_to(Position(1.5, 1.0, 0.0), speed=1.0, accel_linear=1.0, accel_angular=15.0)
        strategy.move_to(Position(1.5, 1.5, 0.0), speed=1.0, accel_linear=1.0, accel_angular=15.0)
        strategy.move_to(Position(1.0, 1.5, 0.0), speed=1.0, accel_linear=1.0, accel_angular=15.0)
        strategy.move_to(Position(1.0, 1.0, 0.0), speed=1.0, accel_linear=1.0, accel_angular=15.0)

    # Come home at the end
    strategy.come_home()

    return strategy

# Example usage
if __name__ == "__main__":
    print(f"Creating strategy with {num_iterations} back-and-forth cycles")
    print()

    # Create the strategy
    strategy = create_main_strategy()

    # Display preview
    print("=== Yellow team strategy ===")
    yellow_dict = strategy.to_dict(Color.YELLOW)
    print(f"Actions: {len(yellow_dict['actions'])}")
    if 'groups' in yellow_dict:
        print(f"Groups: {list(yellow_dict['groups'].keys())}")
    print(f"Init pose: {yellow_dict['init_pose']}")
    print(f"Home pose: {yellow_dict['home_pose']}")

    print("\n=== Blue team strategy ===")
    blue_dict = strategy.to_dict(Color.BLUE)
    print(f"Actions: {len(blue_dict['actions'])}")
    if 'groups' in blue_dict:
        print(f"Groups: {list(blue_dict['groups'].keys())}")
    print(f"Init pose: {blue_dict['init_pose']}")
    print(f"Home pose: {blue_dict['home_pose']}")