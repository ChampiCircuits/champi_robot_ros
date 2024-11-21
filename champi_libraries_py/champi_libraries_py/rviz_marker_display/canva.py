import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from math import cos, sin
import enum


THIN = 0.01
MEDIUM = 0.02
THICK = 0.05


class Item:
    
    def __init__(self):
        self.points_ros = []
        self.type = None
        self.color = None # TODO
        self.scale = None # TODO


    def set_points(self, points: list):
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            if len(p) == 2:
                point_ros.z = 0.
            else:
                point_ros.z = p[2]

            self.points_ros.append(point_ros)
    

    def get_points(self):
        return self.points_ros




class Line(Item):

    def __init__(self, start: tuple, end: tuple, line_width=MEDIUM):
        super().__init__()
        self.set_points([start, end])
        self.type = Marker.LINE_LIST
        self.scale = [line_width, 0., 0.]


class Polyline(Item):

    def __init__(self, points: list, line_width=MEDIUM):
        super().__init__()
        self.set_points(points)
        self.type = Marker.LINE_STRIP
        self.scale = [line_width, 0., 0.]


class Cube(Item):

    def __init__(self, center: tuple, size: tuple[float, float, float], line_width=MEDIUM):
        super().__init()
        


# class Point(Item):
    
#         def __init__(self, point: tuple, scale: float = 0.1):
#             super().__init__()
#             self.set_points([point])
#             self.type = Marker.POINTS
#             self.scale = [scale, 0., 0.]


# class Points(Item):
        
#         def __init__(self, points: list, scale: float = 0.1):
#             super().__init__()
#             self.set_points(points)
#             self.type = Marker.POINTS
#             self.scale = [scale, 0., 0.]


class Rectangle(Polyline):
    
    def __init__(self, center, width, height, angle=0, line_width=MEDIUM):
        
        # Compute the 4 corners of the rectangle
        x = center[0]
        y = center[1]
        w = width
        h = height
        p1 = [cos(angle) * w / 2 - sin(angle) * h / 2 + x, sin(angle) * w / 2 + cos(angle) * h / 2 + y]
        p2 = [-cos(angle) * w / 2 - sin(angle) * h / 2 + x, -sin(angle) * w / 2 + cos(angle) * h / 2 + y]
        p3 = [-cos(angle) * w / 2 + sin(angle) * h / 2 + x, -sin(angle) * w / 2 - cos(angle) * h / 2 + y]
        p4 = [cos(angle) * w / 2 + sin(angle) * h / 2 + x, sin(angle) * w / 2 - cos(angle) * h / 2 + y]

        # Fill Item properties
        super().__init__([p1, p2, p3, p4, p1], width)

    
# TODO layers

class Canva:

    def __init__(self, node, enable: bool):
        self.node = node
        self.enable = enable
        self.items = []

        self.cnt = 0

        self.pub_marker = node.create_publisher(MarkerArray, 'visualization_marker', 10)


    def draw(self):
        
        marker_array = MarkerArray()

        if len(self.items) == 0:
            return

        for item in self.items:
            marker_array.markers.append(self.__item_to_marker(item))
        
        self.pub_marker.publish(marker_array)


    def clear(self):
        self.items = []


    def add(self, item: Item):
        self.items.append(item)
        self.cnt = 0

    
    def __item_to_marker(self, item: Item):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = self.cnt
        marker.type = item.type
        marker.action = Marker.ADD
        marker.points = item.get_points()
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = item.scale[0]
        marker.scale.y = item.scale[1]
        marker.scale.z = item.scale[2]

        self.cnt += 1

        return marker