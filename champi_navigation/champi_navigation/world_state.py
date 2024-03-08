#!/usr/bin/env python3

from champi_navigation import math_bind
from champi_navigation.trajectory import Pose

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
        
class WorldState():
    def __init__(self) -> None:
        self.__self_robot:dict = {"pose":Pose(0, [0., 0.], 0.),
                                 "radius":0.1} #TODO
        self.__opponent_robot:OpponentRobotObject = OpponentRobotObject(1.,1.,0.5,0.5,0.1) #TODO offset
        self.__table:TableObject = TableObject(3.,2.,0.1) #TODO offset
        self.__plants:list[PlantObject] = [] # todo init
        self.__current_state:dict = {"self_robot": self.__self_robot,
                                     "opponent_robot":self.__opponent_robot,
                                     "plants":self.__plants}
        
    def get_table(self) -> TableObject:
        return self.__table
    def get_opponent_robot(self) -> OpponentRobotObject:
        return self.__opponent_robot
    def get_self_robot(self) -> dict:
        return self.__self_robot

    def update(self) -> None:
        # read the ros messages to update current_state
        #   + plants/pots positions eventually
        pass
    def get_current_state(self) -> dict:
        return self.__current_state
