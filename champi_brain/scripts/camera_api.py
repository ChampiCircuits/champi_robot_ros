#!/usr/bin/env python3

import numpy as np
from rclpy.logging import get_logger


def take_picture() -> np.ndarray:
    """Take a picture with the camera and return it as a numpy array."""
    pass

def convert_px_to_mm(plants_positions_px: np.ndarray, supposed_plants_center) -> np.ndarray: # TODO plus besoin de supposed_plants_center
    """Convert the positions of the plants in pixels to mm on the robot frame."""
    # TODO: Implement this function

    # just fake coordinates for now

    # on génère des coords sur un cercle de rayon 10 autour d'un point random
    # on ajoute du bruit sur les coords
    # les n angles sont répartis uniformément + un peu de bruit
    n = 6

    supposed_plants_center += np.random.uniform(-0.01, 0.01, 2)

    coords = []
    for i in range(n):
        angle = 2 * np.pi * i / n + np.random.uniform(-0.2, 0.2)
        r = np.random.uniform(0, 0.05) + 0.250/2. # radius of 250mm, including noise of 5mm max
        x = supposed_plants_center[0] + r * np.cos(angle)
        y = supposed_plants_center[1] + r * np.sin(angle)
        coords.append([x,y])

    get_logger("camera_api").info("\t\tfake generated coords: "+ str(coords))
    return np.array(coords)

def convert_robot_frame_to_world_frame(plants_positions: np.ndarray) -> np.ndarray:
    """Convert the positions of the plants in the robot frame to the world frame."""
    pass