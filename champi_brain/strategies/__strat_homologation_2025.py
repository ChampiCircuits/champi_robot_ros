#!/usr/bin/env python3

"""
Homologation strategy for 2025 competition
Simpler version focusing on banner placement and one element group
"""

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

def create_main_strategy() -> StrategyBuilder:   
    strategy = (StrategyBuilder()
        # Configuration of poses
        .set_init_pose(1.18, 0.16, -60.0)  # yellow starting zone, banner against wall
        .set_home_pose(0.3, 1.0, 0.0)      # yellow ending zone
        .set_wait_to_come_home_pose(0.3, 0.6, 0.0)  # position in front of yellow home zone
        
        # Create groups
        .create_group("banner")
        .create_group("elements_1")
        .create_group("come_home")
        
        ######################### BANNER ###############################################
        # Move forward from starting position
        .move_to(Position(1.18, 0.46, 0.0), group="banner")
        
        ######################### FIRST ELEMENT GROUP ##################################
        # Take elements from first platform
        .take_elements_sequence(Position(1.1, 0.95, 0.0), "elements_1")

        # Put elements
        .put_elements_sequence(Position(1.225, 0.1, 180.0), "elements_1")

        ######################### COME HOME ############################################
        # Move to position to look at aruco marker
        .move_to(Position(0.3, 0.6, -90.0), group="come_home", use_dynamic_layer=True)
        
        # Reset actuators before final positioning
        .custom_action("RESET_ACTUATORS", group="elements_3")
    )
    
    return strategy

# Example usage
if __name__ == "__main__":
    # Create the strategy
    strategy = create_main_strategy()
    
    # Display preview
    print("=== Yellow team strategy (Homologation) ===")
    yellow_dict = strategy.to_dict(Color.YELLOW)
    print(f"Actions: {len(yellow_dict['actions'])}")
    if 'groups' in yellow_dict:
        print(f"Groups: {list(yellow_dict['groups'].keys())}")
    print(f"Init pose: {yellow_dict['init_pose']}")
    print(f"Home pose: {yellow_dict['home_pose']}")
    print()
    
    # Show all actions
    print("Actions:")
    for i, action in enumerate(yellow_dict['actions']):
        print(f"  {i}: {action['action']}", end='')
        if 'target' in action:
            t = action['target']
            print(f" -> ({t['x']:.2f}, {t['y']:.2f}, {t['theta_deg']:.0f}°)", end='')
        if 'group' in action:
            print(f" [group: {action['group']}]", end='')
        print()

    print("\n=== Blue team strategy (Homologation) ===")
    blue_dict = strategy.to_dict(Color.BLUE)
    print(f"Actions: {len(blue_dict['actions'])}")
    if 'groups' in blue_dict:
        print(f"Groups: {list(blue_dict['groups'].keys())}")
    print(f"Init pose: {blue_dict['init_pose']}")
    print(f"Home pose: {blue_dict['home_pose']}")

    print(f"\nStrategy ready for loading with strategy_loader_v2")
