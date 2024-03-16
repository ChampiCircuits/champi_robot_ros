#!/usr/bin/env python3

from champi_navigation import math_bind
from champi_navigation.trajectory import Pose

from geometry_msgs.msg import PoseStamped

from shapely import Point, Polygon
import math

OFFSET = 0.2 # TODO PARAM ROS

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
        self.expanded_poly2 = math_bind.expand(self.polygon, offset+0.03)
        self.offset = offset

        self.dir = 1

    def test_move(self):
        # move up to down and down to up
        if self.center_y > 1.5:
            self.dir = -1
        if self.center_y < 0:
            self.dir = 1
        self.center_y += 0.005*self.dir

        self.polygon = Polygon([Point(self.center_x+self.width/2,self.center_y+self.height/2),
                        Point(self.center_x+self.width/2,self.center_y-self.height/2),
                        Point(self.center_x-self.width/2,self.center_y-self.height/2),
                        Point(self.center_x-self.width/2,self.center_y+self.height/2)])
        self.expanded_poly = math_bind.expand(self.polygon, self.offset)
        self.expanded_poly2 = math_bind.expand(self.polygon, self.offset+0.03)
        # print("moviiiiiiiiiiiiiing")
        # ic("moviiiiiiiiiiiiiing")

class SelfRobot(WorldObject):
    def __init__(self) -> None:
        self.pose_stamped:PoseStamped = PoseStamped() #TODO ptet pas opti?
        self.radius:float = OFFSET #meters

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
        self.opponent_robot:OpponentRobotObject = OpponentRobotObject(center_x=1.5,center_y=0.5,width=0.4,height=0.4,offset=OFFSET)
        self.table:TableObject = TableObject(3.,2.,OFFSET)
        self.plants:list[PlantObject] = [] # todo init
        self.current_state:dict = {"self_robot": self.self_robot,
                                     "opponent_robot":self.opponent_robot,
                                     "plants":self.plants}

    def update(self) -> None:
        self.opponent_robot.test_move()
        # read the ros messages to update current_state
        #   + plants/pots positions eventually
        pass
    def get_current_state(self) -> dict:
        return self.__current_state
