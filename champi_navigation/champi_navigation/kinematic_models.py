import champi_navigation.math_bind as math_bind 

from typing import List
from math import pi, sqrt
from icecream import ic
from shapely import Point, Polygon


class Robot_Kinematic_Model():
    def __init__(self) -> None:
        self.pos = None
        self.wheel_radius = 5.8
        self.wheel_width = 5
        self.robot_radius = 15

        self.linear_speed = [0, 0]  # m/s
        self.angular_speed = 0  # rad/s
        self.has_finished_rotation = False

        self.current_goal = None
        self.goal_reached = True
        self.goals_positions = None  # [[x, y, theta], ...]
        

        self.max_ang_speed = 1.2  # pi/2  # rad/s


        self.graph = None
        self.dico_all_points = {}
        self.path_nodes = None

        self.max_ang_speed = 1.2  # pi/2  # rad/s
    

class Obstacle_static_model():
    def __init__(self, center_x, center_y, width, height, offset) -> None:
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.polygon = Polygon([Point(center_x+width/2,center_y+height/2),
                               Point(center_x+width/2,center_y-height/2),
                               Point(center_x-width/2,center_y-height/2),
                               Point(center_x-width/2,center_y+height/2)])
        self.expanded_obstacle_poly = math_bind.expand(self.polygon, offset)

class Table_static_model():
    def __init__(self, width, height, offset) -> None:
        self.width = width
        self.height = height
        self.polygon = Polygon([Point(0,0),
                                Point(width,0),
                                Point(width, height),
                                Point(0, height)])
        self.expanded_poly = math_bind.expand(self.polygon, -offset)