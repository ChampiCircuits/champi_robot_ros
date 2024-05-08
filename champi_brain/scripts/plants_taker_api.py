#!/usr/bin/env python3

import numpy as np
import yolo_api, camera_api
from sklearn import svm
from typing import Tuple
import time

FRONT_POSE_DIST = 0.3 # m

"""
    # we first have to go in front of the supposed position of the plants
    front_pose_from_plants = plants_taker_api.compute_front_pose_from_plants_position(action["pose"], robot_pose)
    self.robot_navigator.navigate_to(front_pose_from_plants)
    # then we wait to detect the plants
    plants_positions = plants_taker_api.wait_and_detect_plants() # return the 6 positions of the plants
    # we compute the two points trajectory to grab the plants
    trajectory_points = plants_taker_api.compute_trajectory_points(plants_positions, front_pose_from_plants)

"""

def compute_front_pose_from_plants_position(plants_position, robot_pose) -> np.ndarray:
    """Compute a pose to go to in front of the plants."""

    plants_position = np.array(plants_position)
    robot_pose = np.array(robot_pose[:2])

    # we know the robot pose, and the plants central position
    # we create a vector from the robot to the plants
    vector_robot_plants = plants_position - robot_pose
    print("\t\t","plants:",plants_position,"robot:", robot_pose,"vec:", vector_robot_plants)
    # we normalize the vector
    vector_robot_plants = vector_robot_plants / np.linalg.norm(vector_robot_plants)

    # we compute the front pose
    front_pose = plants_position - vector_robot_plants * FRONT_POSE_DIST

    # we add the angle
    angle = np.arctan2(vector_robot_plants[1], vector_robot_plants[0])
    front_pose = np.append(front_pose, angle)

    return front_pose

def convert_robot_frame_to_world_frame(plants_positions: np.ndarray, robot_pose: np.ndarray) -> np.ndarray:
    """Convert the positions of the plants in the robot frame to the world frame using tfs"""

    # transfo_matrix = np.array([[np.cos(robot_pose[2]), -np.sin(robot_pose[2]), robot_pose[0]],
    #                              [np.sin(robot_pose[2]), np.cos(robot_pose[2]), robot_pose[1]],
    #                                 [0, 0, 1]])
    
    # plants_positions = np.hstack([plants_positions, np.ones((plants_positions.shape[0], 1))])
    # plants_positions = np.dot(plants_positions, transfo_matrix.T)

    # TODO remettre

    return plants_positions[:, :2]


def wait_and_detect_plants(supposed_plants_center: np.ndarray, robot_pose: np.ndarray) -> np.ndarray:
    """Wait till x plants are detected in the desired zone. Return the positions of each of the x plants."""

    # TODO pour le moment on prend juste une image, et si y'a pas de plantes, on attend et point

    # we take a picture
    picture = camera_api.take_picture()
    # we detect the plants with yolo
    plants_positions_px = yolo_api.detect_plants(picture)
    # we convert the positions in mm on the robot frame
    plants_positions = camera_api.convert_px_to_mm(plants_positions_px, supposed_plants_center)
    # we convert the positions in the world frame
    plants_positions = convert_robot_frame_to_world_frame(plants_positions, robot_pose)

    # time.sleep(1)

    return plants_positions


def filter_coords(dist, coords, research_zone_center):
    # on garde les coords qui sont à une distance inférieure à dist de la zone de recherche
    new_coords = []
    filtered_out = []
    for coord in coords:
        if np.sqrt((coord[0] - research_zone_center[0])**2 + (coord[1] - research_zone_center[1])**2) < dist:
            new_coords.append(coord)
        else:
            filtered_out.append(coord)
    return new_coords, filtered_out


def compute_line(plants_positions:np.ndarray, front_pose_from_plants: np.ndarray, research_zone_center: np.ndarray) -> Tuple[float, float, np.ndarray, np.ndarray]:
    coord_robot = front_pose_from_plants

    # TODO ajuster la distance de filtrage
    plants_positions, filtered_out = filter_coords(dist=2.0, coords=plants_positions, research_zone_center=research_zone_center)

    print("###################")
    print("###################")
    print(plants_positions, filtered_out)
    print(coord_robot, research_zone_center)

    # Calculer le milieu des six points
    midpoint = np.mean(plants_positions, axis=0)

    # Calculer l'angle entre le robot et le milieu des points
    angle_robot_midpoint = np.arctan2(midpoint[1] - coord_robot[1], midpoint[0] - coord_robot[0])
    
    best_angle_dist = 100000 # best angle c'est l'angle qui est le plus proche de l'angle entre le robot et le milieu des points
    best_angle = 0
    found = False
    for i in range(-180,180):
        # on calcule la droite à l'angle i
        angle = np.radians(i)
        slope = np.tan(angle)
        intercept = midpoint[1] - slope * midpoint[0]

        # on regarde combien de points sont au dessus et en dessous de la droite
        above = []
        below = []
        for coord in plants_positions:
            if coord[1] > slope * coord[0] + intercept:
                above.append(coord)
            else:
                below.append(coord)

        # on regarde si il y a bien trois points de chaque côté
        nb_to_get_each_side = len(plants_positions) // 2
        # print("nb to get each side: ", nb_to_get_each_side)
        if len(above) >= nb_to_get_each_side and len(below) >= nb_to_get_each_side:
            # on regarde si c'est le meilleur angle
            # print("angle: ", np.degrees(angle), "nb above/below: ", len(above), len(below),"angle", abs(angle - angle_robot_midpoint))
            if abs(angle - angle_robot_midpoint) < best_angle_dist:
                best_angle = angle
                # print("\n\n####MEILLEUR ANGLE: ", np.degrees(best_angle))
                found = True
                best_angle_dist = abs(angle - angle_robot_midpoint)

    if not found:
        print("Pas d'angle trouvé")
        quit()
    
    # on calcule la droite à l'angle best_angle
    slope = np.tan(best_angle)
    intercept = midpoint[1] - slope * midpoint[0]

    # on regarde combien de points sont au dessus et en dessous de la droite
    group1 = []
    group2 = []
    for coord in plants_positions:
        if coord[1] > slope * coord[0] + intercept:
            group1.append(coord)
        else:
            group2.append(coord)

    group1 = np.array(group1)
    group2 = np.array(group2)

      
    # SVM pour maximiser les marges entre les deux groupes
    # On utilise un modèle linéaire car on a une droite de séparation
    X = np.vstack([group1, group2])
    y = np.array([0] * len(group1) + [1] * len(group2))
    clf = svm.SVC(kernel='linear')
    clf.fit(X, y)
    slope2 = -clf.coef_[0][0] / clf.coef_[0][1]
    intercept2 = -clf.intercept_ / clf.coef_[0][1]
    intercept2 = intercept2[0]

    return slope2, intercept2, group1, group2

def project_point_on_line(x1, y1, slope, intercept):
    m, b = slope, intercept  # Pente et ordonnée à l'origine de la droite
    print("m: ", m, "b: ", b)
    print("x1: ", x1, "y1: ", y1)
    point = np.array([x1, y1])
    # Calcul du vecteur directeur de la droite
    vecteur_directeur = np.array([1, m])
    
    # Calcul du vecteur de la droite
    vecteur_droite = np.array([0, b])

    # Calcul de la projection orthogonale du point sur la droite
    projection = vecteur_droite + np.dot(point - vecteur_droite, vecteur_directeur) / np.dot(vecteur_directeur, vecteur_directeur) * vecteur_directeur
    print("\t\t\t\tProjection du point sur la droite: ", projection)
    # return np.asarray([18,28.65])
    return projection

def generate_start_end_points(group1, group2, slope2, intercept2, coord_robot):
    # start c'est la projection le point le plus proche du robot sur la droite de séparation
    # end c'est la projection du point le plus loin du robot sur la droite de séparation

    start = [0,0]
    end = [0,0]

    max_dist = 0
    min_dist = 100000
    all_points = np.vstack([group1, group2])

    for point in all_points:
        # dist entre point et robot
        dist = np.sqrt((point[0] - coord_robot[0])**2 + (point[1] - coord_robot[1])**2)
        if dist > max_dist:
            max_dist = dist
            end = point
        if dist < min_dist:
            min_dist = dist
            start = point

    # projection des points sur la droite de séparation
    start_proj = project_point_on_line(start[0], start[1], slope2, intercept2)
    end_proj = project_point_on_line(end[0], end[1], slope2, intercept2)

    # on a besoin de décaler le start en direction du robot d'une distance d
    # on a besoin de décaler le end en direction inverse du robot d'une distance d
    d = 0.2
    # vecteur dir de la droite
    n = np.array([1, slope2])
    n = n / np.linalg.norm(n)

    coord_robot = coord_robot[:2]

    # pour savoir si on doit ajouter ou soustraire d a start
    if np.dot(n, start_proj - coord_robot) > 0:
        start_proj = start_proj - d * n
    else:
        start_proj = start_proj + d * n
    
    # pour savoir si on doit ajouter ou soustraire d a end
    if np.dot(n, end_proj - coord_robot) > 0:
        end_proj = end_proj + d * n
    else:
        end_proj = end_proj - d * n

    print("\t\t\tstart: ", start_proj)
    print("\t\t\tend: ", end_proj)
    
    return start_proj, end_proj

def compute_trajectory_points(plants_positions:np.ndarray, front_pose_from_plants: np.ndarray, research_zone_center: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """ compute the trajectory points to grab the plants.
        returns the start and end points of the trajectory as [x, y, theta] numpy arrays.
    """
    
    # We have the x positions of the plants, we want to compute the two points trajectory to grab the plants
    # We first compute the line that separates the plants in two groups
    # time.sleep(2)
    print("\t\tcompute line...")
    slope, intercept, group1, group2 = compute_line(plants_positions, front_pose_from_plants, research_zone_center)

    # We compute the two points
    print("\t\tgenerate start and end points...")
    start, end = generate_start_end_points(group1, group2, slope, intercept, front_pose_from_plants)

    # start, end are only x,y coordinates, we need to add the theta
    # we compute the angle between the two points
    angle = np.arctan2(end[1] - start[1], end[0] - start[0])

    # we add the angle to the points
    start = np.append(start, angle)
    end = np.append(end, angle)

    print("\t\tstart: ", start, "end: ", end)

    return start, end