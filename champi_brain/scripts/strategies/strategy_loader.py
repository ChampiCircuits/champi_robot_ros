#!/usr/bin/env python3

import yaml, math
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def load_strategy(file_path, logger): # only one recursion level in files
    data = load_yaml(file_path)

    init_pose = [data['init_pose']['x'], data['init_pose']['y'], data['init_pose']['theta_deg']+90.0]  # +90° to align with the coordinate system
    logger.info(f'<< Init pose will be {init_pose[0]} {init_pose[1]} {init_pose[2]}°!')

    actions = []

    for (i, action) in enumerate(data['actions']):
        logger.info(f'Action {i}: {action}')

        if action['action'] == 'include_sub_file':
            sub_file_name = action['file']
            logger.info(f'SUB {i}: {sub_file_name}')
            sub_strat = load_yaml(get_package_share_directory('champi_brain') + '/scripts/strategies/sub/' + sub_file_name)
            tag = action['parameters']['tag']

            # for transforming coordinates
            action_coords = action['parameters']['target']
            x_action, y_action, theta_deg_action = action_coords['x'], action_coords['y'], action_coords['theta_deg']

            for (j, sub_action) in enumerate(sub_strat['actions']):
                logger.info(f'Action {i}.{j}: {sub_action}')

                if sub_action['action'] in ['move', 'detectPlatform']: # moveForPlatform are not yet transformed
                    sub_action = apply_transformation(sub_action, x_action, y_action, theta_deg_action)

                sub_action['tag'] = tag
                actions.append(sub_action)
        else:
            actions.append(action)

    logger.info(f'\n\n')
    for (i, action) in enumerate(actions):
        logger.info(f'Action {i}: {action}')

    return actions, init_pose

def apply_transformation(sub_action, x_action, y_action, theta_deg_action):

    # Extract sub_action coordinates
    x_sub = sub_action['target']['x']
    y_sub = sub_action['target']['y']
    theta_deg_sub = sub_action['target']['theta_deg']

    # Convert angles to radians
    theta_rad = math.radians(theta_deg_action)

    # Apply rotation and translation
    x_transformed = (x_sub * math.cos(theta_rad) - y_sub * math.sin(theta_rad)) + x_action
    y_transformed = (x_sub * math.sin(theta_rad) + y_sub * math.cos(theta_rad)) + y_action
    theta_transformed = (theta_deg_sub + theta_deg_action) % 360

    # Update sub_action with transformed coordinates
    sub_action['target']['x'] = x_transformed
    sub_action['target']['y'] = y_transformed
    sub_action['target']['theta_deg'] = theta_transformed

    return sub_action

