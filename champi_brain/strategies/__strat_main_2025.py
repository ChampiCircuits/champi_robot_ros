#!/usr/bin/env python3

from champi_brain.strategy_dsl import StrategyBuilder, Position, Color

def create_main_strategy() -> StrategyBuilder:   
    strategy = (StrategyBuilder()
        # Configuration of poses
        .set_init_pose(1.18, 0.16, -60.0)  # yellow starting zone
        .set_home_pose(0.3, 1.8, 0.0)      # yellow ending zone
        
        # Create groups
        .create_group("banner")
        .create_group("elements_1")
        .create_group("elements_2")
        .create_group("elements_3")
        .create_group("come_home")
        
        ######################### BANNER ###############################################
        .put_banner(1.18, 0.16, 0.0, "banner")
        .move_to(1.18, 0.46, 0.0, group="banner", 
                end_speed=0.2, speed=1.0, accel_linear=1.1, accel_angular=15.0)
        
        ######################### FIRST ELEMENT GROUP ################################
        .get_ready("elements_1")
        .take_elements_sequence(1.1, 0.95, 0.0, "elements_1")
        
        # Intermediate movement
        .move_to(1.225, 0.6, 180.0, group="elements_1",
                speed=1.0, accel_linear=1.0, accel_angular=12.0)
        
        .put_elements_sequence(1.225, 0.1, 180.0, "elements_1")
        
        ######################### SECOND ELEMENT GROUP ###############################
        .get_ready("elements_2")
        .move_to(0.9, 0.75, 90.0, group="elements_2", use_dynamic_layer=True,
                speed=1.0, accel_linear=1.0, accel_angular=15.0)
        .take_elements_sequence(0.775, 0.25, 180.0, "elements_2")
        .put_elements_sequence(1.225, 0.28, 180.0, "elements_2")
        
        ######################### THIRD ELEMENT GROUP ################################
        .get_ready("elements_3")
        .move_to(0.9, 0.6, 90.0, group="elements_3", use_dynamic_layer=True,
                speed=1.0, accel_linear=1.0, accel_angular=15.0)
        .take_elements_sequence(0.075, 0.4, 90.0, "elements_3")
        .put_elements_sequence(0.6, 0.1, 180.0, "elements_3")
        
        ######################### COME HOME ##########################################
        .move_to(0.45, 0.9, 180.0, group="come_home", use_dynamic_layer=True,
                speed=1.0, accel_linear=1.0, accel_angular=15.0)

        # .come_home() # done automatically at the end by the planner
    )
    
    return strategy

# Example usage
if __name__ == "__main__":
    # Create the strategy
    strategy = create_main_strategy()
    
    # Display preview
    print("=== Yellow team strategy ===")
    yellow_dict = strategy.to_dict(Color.YELLOW)
    print(f"Actions: {len(yellow_dict['actions'])}")
    print(f"Groups: {list(yellow_dict['groups'].keys())}")
    print(f"Init pose: {yellow_dict['init_pose']}")
    print(f"Home pose: {yellow_dict['home_pose']}")

    print("\n=== Blue team strategy ===")
    blue_dict = strategy.to_dict(Color.BLUE)
    print(f"Actions: {len(blue_dict['actions'])}")
    print(f"Groups: {list(blue_dict['groups'].keys())}")
    print(f"Init pose: {blue_dict['init_pose']}")
    print(f"Home pose: {blue_dict['home_pose']}")

    print(f"\nStrategy ready for loading with strategy_loader_v2")
