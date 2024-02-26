import champi_navigation.math_bind as math_bind 

from typing import List
from math import pi, sqrt
from icecream import ic
from shapely import Point, Polygon


class Robot_Kinematic_Model():
    def __init__(self) -> None:

        self.linear_speed = [0, 0]  # m/s
        self.angular_speed = 0  # rad/s
    

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