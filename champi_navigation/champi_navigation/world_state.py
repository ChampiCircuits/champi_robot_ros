#!/usr/bin/env python3

from champi_navigation import math_bind
from champi_navigation.trajectory import Pose

from geometry_msgs.msg import PoseStamped

from shapely import Point, Polygon

class WorldObject():
    def __init__(self) -> None:
        self.__pose:Pose = None
        # updated to 1 when the object is detected. Slowly decreases over time. 
        self.__position_certainty = 0

class OpponentRobotObject(WorldObject):
    def __init__(self, center_x, center_y, width, height, offset) -> None:
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.polygon = Polygon([Point(center_x+width/2,center_y+height/2),
                               Point(center_x+width/2,center_y-height/2),
                               Point(center_x-width/2,center_y-height/2),
                               Point(center_x-width/2,center_y+height/2)])
        self.expanded_poly = math_bind.expand(self.polygon, offset)

class SelfRobot(WorldObject):
    def __init__(self) -> None:
        self.pose_stamped:PoseStamped = PoseStamped() #TODO ptet pas opti?
        self.radius:float = 0.1 #TODO param

class PlantObject(WorldObject):
    def __init__(self) -> None:
        self.__is_standing:bool = True


class TableObject(WorldObject):
    def __init__(self, width, height, offset) -> None:
        self.width = width
        self.height = height
        self.polygon = Polygon([Point(0,0),
                                Point(width,0),
                                Point(width, height),
                                Point(0, height)])
        self.expanded_poly = math_bind.expand(self.polygon, -offset)
        self.expanded_poly = math_bind.expand(self.polygon, offset) # TODO JUSTE POUR DEBUG
        
class WorldState():
    def __init__(self) -> None:
        self.self_robot:SelfRobot = SelfRobot()
        self.opponent_robot:OpponentRobotObject = OpponentRobotObject(0.5,0.5,0.5,0.5,0.1) #TODO offset
        self.table:TableObject = TableObject(3.,2.,0.1) #TODO offset
        self.plants:list[PlantObject] = [] # todo init
        self.current_state:dict = {"self_robot": self.self_robot,
                                     "opponent_robot":self.opponent_robot,
                                     "plants":self.plants}

    def update(self) -> None:
        # read the ros messages to update current_state
        #   + plants/pots positions eventually
        pass
    def get_current_state(self) -> dict:
        return self.__current_state
