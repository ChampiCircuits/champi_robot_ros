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




class MarkerArrayDecorator:
    
    def __init__(self, nb_markers):
        self.markers = []
        for i in range(nb_markers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = Marker.ADD
            self.markers.append(marker)


    def __set_random_color(self):
        r = random.random()
        g = random.random()
        b = random.random()
        for marker in self.markers:
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.
    
    def set_color(self, color: tuple):
        if color is None:
            self.__set_random_color()
        else :
            for marker in self.markers:
                marker.color.r = color[0] / 255
                marker.color.g = color[1] / 255
                marker.color.b = color[2] / 255
                marker.color.a = 1.
    

    def __set_points_from_tuples_2d(self, points: list):
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            point_ros.z = 0.
            self.markers[0].points.append(point_ros) # TODO make reliable
    

    def __set_points_from_tuples_3d(self, points: list):
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            point_ros.z = float(p[2])
            self.markers[0].points.append(point_ros) # TODO make reliable


    def set_points(self, points: list):
        """accepts a 2D tuples, 3D tuples or list of ROS Points"""        

        if len(points) == 0:
            return
        
        if isinstance(points[0], Point):
            self.points = points

        elif isinstance(points[0], Pose):
            for pose in points:
                self.markers[0].points.append(pose.position)

        elif len(points[0]) == 2:
            self.__set_points_from_tuples_2d(points)

        elif len(points[0]) == 3:
            self.__set_points_from_tuples_3d(points)
        else:
            raise ValueError("Invalid points")
    

    def set_pose(self, pose, index=0):

        if isinstance(pose, Pose):
            self.markers[index].pose = pose
        elif isinstance(pose, Point):
            self.__set_pose_from_point(pose, index)
        elif len(pose) == 2:
            self.__set_pose_from_tuple_2d(pose, index)
        elif len(pose) == 3:
            self.__set_pose_from_tuple_3d(pose, index)
        else:
            raise ValueError("Invalid pose")


    def __set_pose_from_tuple_3d(self, pose, index=0):
        self.markers[index].pose.position.x = float(pose[0])
        self.markers[index].pose.position.y = float(pose[1])
        self.markers[index].pose.position.z = float(pose[2])
        self.markers[index].pose.orientation.w = 1.
    
    def __set_pose_from_tuple_2d(self, pose, index=0):
        self.markers[index].pose.position.x = float(pose[0])
        self.markers[index].pose.position.y = float(pose[1])
        self.markers[index].pose.position.z = 0.
        self.markers[index].pose.orientation.w = 1.
    
    
    def __set_pose_from_point(self, point, index=0):
        self.markers[index].pose.position = point
        self.markers[index].pose.orientation.w = 1.
    

    def set_poses(self, poses):
        if len(poses) == 0:
            return

        for i, pose in enumerate(poses):
            self.set_pose(pose, index=i)


    def set_scale(self, scale):
        if isinstance(scale, float):
            scale = [scale, scale, scale]
        for marker in self.markers:
            marker.scale.x = float(scale[0])
            marker.scale.y = float(scale[1])
            marker.scale.z = float(scale[2])


    def get_marker_array(self):
        return self.markers


    def set_type(self, type):
        for marker in self.markers:
            marker.type = type
        

class Polyline(MarkerArrayDecorator):
    
    def __init__(self, poses, size=MEDIUM, color=None):
        super().__init__(1)
        self.set_type(Marker.LINE_STRIP)
        self.set_scale([size, 0., 0.])
        self.set_points(poses)
        self.set_color(color)


class OrientedCube(MarkerArrayDecorator):
        
    def __init__(self, pose, size, color=None):
        super().__init__(1)
        self.set_type(Marker.CUBE)
        self.set_scale(size)
        self.set_pose(pose)
        self.set_color(color)


class Sphere(MarkerArrayDecorator):
            
    def __init__(self, pose, size, color=None):
        super().__init__(1)
        self.set_type(Marker.SPHERE)
        self.set_scale(size)
        self.set_pose(pose)
        self.set_color(color)


class Arrow(MarkerArrayDecorator):
            
    def __init__(self, pose, size=0.5, color=None):
        super().__init__(1)
        self.set_type(Marker.ARROW)
        self.set_scale([size, size/5., size/5.])
        self.set_pose(pose)
        self.set_color(color)


class Arrows(MarkerArrayDecorator):
    
        def __init__(self, poses, size=0.5, color=None):
            super().__init__(len(poses))
            for pose in poses:
                arrow = Arrow(pose, size, color)
                self.markers.extend(arrow.get_marker_array())



class Spheres(MarkerArrayDecorator):
        
    def __init__(self, poses, size=0.2, color=None):
        super().__init__(1)
        self.set_type(Marker.SPHERE_LIST)
        self.set_scale(size)
        self.set_points(poses)
        self.set_color(color)


class Cubes(MarkerArrayDecorator):
            
    def __init__(self, poses, size=0.2, color=None):
        super().__init__(1)
        self.set_type(Marker.CUBE_LIST)
        self.set_scale(size)
        self.set_points(poses)
        self.set_color(color)


class OrientedCubes(MarkerArrayDecorator):
    
    def __init__(self, poses, size=0.2, color=None):
        super().__init__(len(poses))
        for pose in poses:
            cube = OrientedCube(pose, size, color)
            self.markers.extend(cube.get_marker_array())


class Points(MarkerArrayDecorator):
            
    def __init__(self, poses, size=MEDIUM, color=None):
        super().__init__(1)
        self.set_type(Marker.POINTS)
        self.set_scale(size)
        self.set_points(poses)
        self.set_color(color)


class Poses(MarkerArrayDecorator):
        

    class Types(enum.Enum):
        POINTS = 1
        CUBES = 2
        SPHERES = 3
        ARROWS = 4
        ORIENTED_CUBES = 5
        
    def __init__(self, poses, size=0.2, color=None, type=Types.ARROWS):
        
        super().__init__(0)
        if type == Poses.Types.POINTS:
            points = Points(poses, size, color)
            self.markers.extend(points.get_marker_array())
        elif type == Poses.Types.CUBES:
            cubes = Cubes(poses, size, color)
            self.markers.extend(cubes.get_marker_array())
        elif type == Poses.Types.SPHERES:
            spheres = Spheres(poses, size, color)
            self.markers.extend(spheres.get_marker_array())
        elif type == Poses.Types.ARROWS:
            arrows = Arrows(poses, size, color)
            self.markers.extend(arrows.get_marker_array())
        elif type == Poses.Types.ORIENTED_CUBES:
            oriented_cubes = OrientedCubes(poses, size, color)
            self.markers.extend(oriented_cubes.get_marker_array())
        else:
            raise ValueError("Invalid type")
            
                
                


# class Points(MarkerWrapper):
        
#     def __init__(self, poses, size=MEDIUM, color=None):
#         super().__init__()
#         self.type = Marker.POINTS
#         self.scale.x = size
#         self.scale.y = size
#         self.scale.z = size
#         self.set_points(poses)
#         self.set_color(color)


# class Poses(MarkerArrayDecorator):

#     class Types(enum.Enum):
#         POINTS = 1
#         CUBES = 2
#         SPHERES = 3
#         ARROWS = 4
#         FRAMES = 5
    
#     class Sizes(enum.Enum):
#         SMALL = 0.1
#         MEDIUM = 0.2
#         LARGE = 0.3
    
#     def __init__(self, poses, size=Sizes.LARGE, type=Types.POINTS, color=None):

#         if type == Poses.Types.POINTS:
#             super().__init__(1)
#             self.set_type(Marker.POINTS)
#         elif type == Poses.Types.CUBES:
#             super().__init__(1)
#             self.set_type(Marker.CUBE_LIST)
#         elif type == Poses.Types.SPHERES:
#             super().__init__(1)
#             self.set_type(Marker.SPHERE_LIST)
#         elif type == Poses.Types.ARROWS:
#             super().__init__(len(poses))
#             self.set_type(Marker.ARROW)
#         elif type == Poses.Types.FRAMES:
#             pass #TODO
#         else:
#             raise ValueError("Invalid type")
        
#         self.set_scale([size, size, size])
#         self.set_color(color)
        



class Canva:

    def __init__(self, node, enable: bool):
        self.node = node
        self.enable = enable
        self.items = []

        self.pub_marker = node.create_publisher(MarkerArray, 'visualization_marker', 10)

        self.nb_markers = 0


    def draw(self):
        
        if len(self.items) == 0:
            return
        
        marker_array = MarkerArray()

        for item in self.items:
            for marker in item.get_marker_array():
                marker_array.markers.append(marker)
        
        for i, marker in enumerate(marker_array.markers):
            marker.id = i
        
        self.pub_marker.publish(marker_array)

        self.nb_markers = len(marker_array.markers)


    def clear(self):
        self.items = []
        marker_array = MarkerArray()
        for i in range(self.nb_markers):
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = i
            marker_array.markers.append(marker)
            

    def add(self, item):
        self.items.append(item)