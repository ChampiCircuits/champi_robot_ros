#!/usr/bin/env python3

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from rclpy.duration import Duration

from typing import Tuple
from math import cos, sin
from enum import Enum

# state machine for taking plants
class StateTakingPlants(Enum):
    COMPUTE_FRONT_POSE = 0
    NAVIGATE_TO_FRONT_POSE = 1
    DETECT_PLANTS = 2
    COMPUTE_TRAJECTORY = 3
    NAVIGATE_TO_TRAJECTORY_1 = 4
    NAVIGATE_TO_TRAJECTORY_2 = 5

# enum for the main state machine
class State(Enum):
    INIT = 0
    NO_ACTION = 1
    IN_ACTION = 2
    PLANTES = 3
    POSE_PLANTES = 4
    PANNEAU = 5
    RETOUR = 6
    ACTION_FINISHED = 7
    JUST_MOVE = 8
    NOT_STARTED = 9

class CAN_MSGS(Enum):
    START_GRAB_PLANTS = 0
    STOP_GRAB_PLANTS = 1
    RELEASE_PLANT = 2
    TURN_SOLAR_PANEL = 3
    INITIALIZING = 4
    FREE = 5

OFFSET_POSE_PLANT_X = 0.08
OFFSET_POSE_PLANT_Y = 0.08

def draw_rviz_markers(strategy_node, positions, marker_namespace, color, size):
    marker_array = MarkerArray()
    
    i=0
    for position in positions:
        marker = Marker()
        marker.header.frame_id = 'odom'  # Cadre de référence
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position du marqueur
        position = [position[0], position[1]]
        marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=0.0)

        # Orientation du marqueur
        marker.pose.orientation.w = 1.0

        # Taille du cercle
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        
        # Couleur du cercle
        marker.color = color  # Vert avec pleine opacité
        
        # Durée du marqueur
        marker.lifetime = Duration(seconds=500).to_msg()  # 5 secondes
        
        # Identifiant unique pour chaque marqueur
        marker.id = i
        i+=1
        
        # Ajouter le marqueur à l'array
        marker_array.markers.append(marker)

    # if str begins with /markers_plants_detected/*
    if marker_namespace.startswith("/markers_plants_detected"):
        strategy_node.markers_publisher_plants.publish(marker_array)
    elif marker_namespace.startswith("/markers_selected_action"):
        strategy_node.markers_publisher_circles.publish(marker_array)

def calc_time_of_travel(pose1: Tuple[float, float], pose2: Tuple[float, float], ROBOT_MEAN_SPEED:float) -> float:
    """Calculate the time of travel between two points"""
    distance = ((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2)**0.5
    return distance / ROBOT_MEAN_SPEED


def get_action_by_name(name: str, actions: dict) -> dict:
    for action in actions:
        if action["name"] == name:
            return action
    return None

def pose_from_position(position, stamp):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = stamp
    goal_pose.pose.position.x = position[0]
    goal_pose.pose.position.y = position[1]
    goal_pose.pose.position.z = 0.0
    # theta radians to quaternion
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.z = sin(position[2] / 2)
    goal_pose.pose.orientation.w = cos(position[2] / 2)
    return goal_pose

def pose_with_cov_from_position(position, stamp):
    goal_pose = PoseWithCovarianceStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = stamp
    goal_pose.pose.pose.position.x = position[0]
    goal_pose.pose.pose.position.y = position[1]
    # theta radians to quaternion
    goal_pose.pose.pose.orientation.z = sin(position[2] / 2)
    goal_pose.pose.pose.orientation.w = cos(position[2] / 2)
    return goal_pose
