import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Pose

from math import cos, sin
import enum

import random

import champi_libraries_py.rviz_marker_display.colors as colors

from icecream import ic

THIN = 0.01
MEDIUM = 0.02
THICK = 0.05




class MarkerWrapper(Marker):
    
    def __init__(self):
        super().__init__()

    
    def __set_random_color(self):
        self.color.r = random.random()
        self.color.g = random.random()
        self.color.b = random.random()
        self.color.a = 1.
    
    def set_color(self, color: tuple):
        if color is None:
            self.__set_random_color()
        else :
            self.color.r = color[0] / 255.
            self.color.g = color[1] / 255.
            self.color.b = color[2] / 255.
            self.color.a = 1.
    

    def __set_points_from_tuples_2d(self, points: list):
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            point_ros.z = 0.
            self.points.append(point_ros)
    

    def __set_points_from_tuples_3d(self, points: list):
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            point_ros.z = float(p[2])
            self.points.append(point_ros)

    def set_points(self, points: list):
        """accepts a 2D tuples, 3D tuples or list of ROS Points"""        

        if len(points) == 0:
            return
        
        if isinstance(points[0], Point):
            self.points = points

        elif isinstance(points[0], Pose):
            for pose in points:
                self.points.append(pose.position)

        elif len(points[0]) == 2:
            self.__set_points_from_tuples_2d(points)

        elif len(points[0]) == 3:
            self.__set_points_from_tuples_3d(points)
        else:
            raise ValueError("Invalid points")
    

    def set_pose(self, pose):

        if isinstance(pose, Pose):
            self.pose = pose
        elif isinstance(pose, Point):
            self.__set_pose_from_point(pose)
        elif len(pose) == 2:
            self.__set_pose_from_tuple_2d(pose)
        elif len(pose) == 3:
            self.__set_pose_from_tuple_3d(pose)
        else:
            raise ValueError("Invalid pose")


    def __set_pose_from_tuple_3d(self, pose):
        self.pose.position.x = float(pose[0])
        self.pose.position.y = float(pose[1])
        self.pose.position.z = float(pose[2])
        self.pose.orientation.w = 1.
    
    def __set_pose_from_tuple_2d(self, pose):
        self.pose.position.x = float(pose[0])
        self.pose.position.y = float(pose[1])
        self.pose.position.z = 0.
        self.pose.orientation.w = 1.
    
    
    def __set_pose_from_point(self, point):
        self.pose.position = point
        self.pose.orientation.w = 1.


    def get_base_marker(self):
        """Necessary, because the publish method checks that argument is strictly a Marker

        Returns:
            _type_: _description_
        """
        marker = Marker() # TODO use decorator instead ?
        # Manually copy all attributes from the current object to the new one
        for field in Marker.get_fields_and_field_types().keys():
            setattr(marker, field, getattr(self, field))
        return marker
        

    


class Line(MarkerWrapper):
    
    def __init__(self, start, end, line_width=MEDIUM, color = None):
        super().__init__()
        self.type = Marker.LINE_LIST
        self.scale.x = float(line_width)
        self.scale.y = 0.
        self.scale.z = 0.
        self.set_points([start, end])
        self.set_color(color)



class Polyline(MarkerWrapper):
    
    def __init__(self, points, line_width=MEDIUM, color=None):
        super().__init__()
        self.type = Marker.LINE_STRIP
        self.scale.x = line_width
        self.scale.y = 0.
        self.scale.z = 0.
        self.set_points(points)
        self.set_color(color)


class Cube(MarkerWrapper):
        
    def __init__(self, pose, size, color=None):
        super().__init__()
        self.type = Marker.CUBE
        self.scale.x = float(size[0])
        self.scale.y = float(size[1])
        self.scale.z = float(size[2])
        self.set_pose(pose)
        self.set_color(color)


class Sphere(MarkerWrapper):
            
    def __init__(self, pose, radius, color=None):
        super().__init__()
        self.type = Marker.SPHERE
        self.scale.x = float(radius)
        self.scale.y = float(radius)
        self.scale.z = float(radius)
        self.set_pose(pose)
        self.set_color(color)


# class Points(MarkerWrapper):
        
#     def __init__(self, poses, size=MEDIUM, color=None):
#         super().__init__()
#         self.type = Marker.POINTS
#         self.scale.x = size
#         self.scale.y = size
#         self.scale.z = size
#         self.set_points(poses)
#         self.set_color(color)


class Poses(MarkerWrapper):

    class Types(enum.Enum):
        POINTS = 1
        CUBES = 2
        SPHERES = 3
        ARROWS = 4
        FRAMES = 5
    
    class Sizes(enum.Enum):
        SMALL = 0.1
        MEDIUM = 0.2
        LARGE = 0.3
    
    def __init__(self, poses, size=Sizes.LARGE, type=Types.POINTS, color=None):
        super().__init__()
        self.scale.x = size
        self.scale.y = size
        self.scale.z = size
        if type == Poses.Types.POINTS:
            self.type = Marker.POINTS
        elif type == Poses.Types.CUBES:
            self.type = Marker.CUBE_LIST
        elif type == Poses.Types.SPHERES:
            self.type = Marker.SPHERE_LIST
        elif type == Poses.Types.ARROWS:
            self.type = Marker.ARROW
        elif type == Poses.Types.FRAMES:
            pass #TODO
        else:
            raise ValueError("Invalid type")


        self.set_points(poses)
        self.set_color(color)
        



class Canva:

    def __init__(self, node, enable: bool):
        self.node = node
        self.enable = enable
        self.markers = []

        self.cnt = 0

        self.pub_marker = node.create_publisher(MarkerArray, 'visualization_marker', 10)


    def draw(self):
        
        if len(self.markers) == 0:
            return
        
        marker_array = MarkerArray()
        for marker in self.markers:
            marker.header.frame_id = 'map'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.action = Marker.ADD
            marker.id = self.cnt
            marker_array.markers.append(marker.get_base_marker())
            self.cnt += 1
        
        self.pub_marker.publish(marker_array)

    def clear(self):
        self.markers = []
        self.cnt = 0


    def add(self, marker: Marker):
        self.markers.append(marker)