#!/usr/bin/env python3

import yaml, math
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def load_init_or_home_pose(name_str, data, color):
    init_or_home_pose = [data[name_str]['x'], data[name_str]['y'], data[name_str]['theta_deg']]
    if color == 'BLUE': # transform x if blue through vertical axis
        init_or_home_pose[0] = 3.0 - init_or_home_pose[0] # inverse the
        # init_or_home_pose[2] = (360 - init_or_home_pose[2]) % 360 # inverse the angle
        # do not inverse the angle, because it is already in the correct direction to put banner

    init_or_home_pose[2] = init_or_home_pose[2] +90.0  # +90° to align with the coordinate system
    return init_or_home_pose


def load_strategy(file_path, color, logger): # only one recursion level in files
    data = load_yaml(file_path)
    init_pose = load_init_or_home_pose('init_pose', data, color)
    logger.info(f'<< Init pose will be {init_pose[0]} {init_pose[1]} {init_pose[2]}°!')
    home_pose = load_init_or_home_pose('home_pose', data, color)
    logger.info(f'<< Home pose will be {home_pose[0]} {home_pose[1]} {home_pose[2]}°!')

    actions = []

    for (i, action) in enumerate(data['actions']):
        logger.info(f'Action {i}: {action}')

        if color == 'BLUE': # transform x if blue through vertical axis
            if 'target' in action:
                action['target']['x'] = 3.0 - action['target']['x']
                action['target']['theta_deg'] = (360 - action['target']['theta_deg']) % 360
            if 'parameters' in action and 'target' in action['parameters']:
                action['parameters']['target']['x'] = 3.0 - action['parameters']['target']['x']
                action['parameters']['target']['theta_deg'] = (360 - action['parameters']['target']['theta_deg']) % 360

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

    return actions, init_pose, home_pose

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

