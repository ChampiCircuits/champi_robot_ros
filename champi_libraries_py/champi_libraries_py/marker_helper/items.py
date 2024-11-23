from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Path

import champi_libraries_py.marker_helper.presets as presets
import champi_libraries_py.marker_helper.shared_variables as shared_variables

import champi_libraries_py.data_types.geometry as geo



class Item:

    """It is a decorator, of a an array of markers. Methods like set_color, apply to all markers of the marker array.
    However, some methods like set_points apply to only the first element array. This is because such method only make
    sense for items composed of only one Marker.
    Both types of methods are identified in their description as "ONE_MARKER" or "ALL_MARKERS"
    It does not make sense to call a ONE_MARKER method if you have a several-markers Item, however, ALL_MARKERS methods
    can be used for a one-marker Item.

    Unless stated otherwise, method arguments points, poses or poses can be of the following types:
    - tuple(x, y): the z coordinate considered 0. If it is a pose the orientation is considered quat(0, 0, 0, 1)
    - tuple (x, y, z).
    - Point(ROS type)
    - Pose (ROS type)
    - PoseWithCovariance
    - PoseWithCovarianceStamped
    - geometry.Pose2D

    """
    
    def __init__(self, nb_markers):
        self.markers = []
        for i in range(nb_markers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = Marker.ADD
            self.markers.append(marker)


    def __set_color_auto(self):
        """ALL_MARKERS. Set the color of the Item to a color from the presets.__COLOR_LIST.
        """
        i = shared_variables.i_color % len(presets.COLOR_LIST)
        shared_variables.i_color += 1
        self.set_color(presets.COLOR_LIST[i])
    
    def set_color(self, color: tuple):
        """ALL_MARKERS. Set the color of the Item.

        Args:
            color (tuple): color is a tuple of 3 elements: (r, g, b), with values between 0 and 255.
        """
        if color is None:
            self.__set_color_auto()
        else :
            for marker in self.markers:
                marker.color.r = color[0] / 255
                marker.color.g = color[1] / 255
                marker.color.b = color[2] / 255
                marker.color.a = 1.
    

    def __set_points_from_tuples_2d(self, points: list):
        """ONE_MARKER. Set the "points" attribut of the marker.

        Args:
            points (list): a point is expressed as a tuple(x, y). z is set to 0.
        """
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            point_ros.z = 0.
            self.markers[0].points.append(point_ros)
    

    def __set_points_from_tuples_3d(self, points: list):
        """ONE_MARKER. Set the "points" attribut of the marker.

        Args:
            points (list): a point is expressed as a tuple(x, y, z).
        """
        for p in points:
            point_ros = Point()
            point_ros.x = float(p[0])
            point_ros.y = float(p[1])
            point_ros.z = float(p[2])
            self.markers[0].points.append(point_ros)


    def set_points(self, points: list):
        """ONE_MARKER. Set the "points" attribut of the marker.

        Args:
            points (list): points are of any type listed in class description.
        """  
        if len(points) == 0:
            return
        
        if isinstance(points[0], Point):
            self.points = points
        elif isinstance(points[0], Pose):
            for pose in points:
                self.markers[0].points.append(pose.position)
        elif isinstance(points[0], PoseWithCovariance):
            for pose in points:
                self.markers[0].points.append(pose.pose.position)
        elif isinstance(points[0], PoseWithCovarianceStamped):
            for pose in points:
                self.markers[0].points.append(pose.pose.pose.position)
        elif isinstance(points, Path):
            for pose in points.poses:
                self.markers[0].points.append(pose.pose.position)
        
        elif isinstance(points[0], geo.Pose2D):
            for pose in points:
                self.markers[0].points.append(pose.to_ros_pose().position)

        elif len(points[0]) == 2:
            self.__set_points_from_tuples_2d(points)

        elif len(points[0]) == 3:
            self.__set_points_from_tuples_3d(points)

        else:
            raise ValueError("Invalid points")
    

    def set_pose(self, pose, index=0):
        """ONE_MARKER. Set the "pose" attribut of the marker.

        Args:
            pose: pose is of any type listed in class description.
        """  
        if isinstance(pose, Pose):
            self.markers[index].pose = pose
        elif isinstance(pose, Point):
            self.__set_pose_from_point(pose, index)
        elif isinstance(pose, PoseWithCovariance):
            self.markers[index].pose = pose.pose
        elif isinstance(pose, PoseWithCovarianceStamped):
            self.markers[index].pose = pose.pose.pose

        elif isinstance(pose, geo.Pose2D):
            self.markers[index].pose = pose.to_ros_pose()

        elif len(pose) == 2:
            self.__set_pose_from_tuple_2d(pose, index)
        elif len(pose) == 3:
            self.__set_pose_from_tuple_3d(pose, index)

        else:
            raise ValueError("Invalid pose")


    def __set_pose_from_tuple_3d(self, pose, index=0):
        """ONE_MARKER. Set the "pose" attribut of the marker.

        Args:
            pose: A tuple(x, y). z is set to 0.
        """
        self.markers[index].pose.position.x = float(pose[0])
        self.markers[index].pose.position.y = float(pose[1])
        self.markers[index].pose.position.z = float(pose[2])
        self.markers[index].pose.orientation.w = 1.
    
    def __set_pose_from_tuple_2d(self, pose, index=0):
        """ONE_MARKER. Set the "pose" attribut of the marker.

        Args:
            pose: A tuple(x, y, z).
        """
        self.markers[index].pose.position.x = float(pose[0])
        self.markers[index].pose.position.y = float(pose[1])
        self.markers[index].pose.position.z = 0.
        self.markers[index].pose.orientation.w = 1.
    
    
    def __set_pose_from_point(self, point, index=0):
        """ONE_MARKER. Set the "pose" attribut of the marker.

        Args:
            pose: A ROS Point.
        """
        self.markers[index].pose.position = point
        self.markers[index].pose.orientation.w = 1.
    

    def set_poses(self, poses):
        """ALL_MARKERS. Set the "pose" of each marker.

        Args:
            poses(list): poses are of any type listed in class description.
        """
        if len(poses) == 0:
            return
        
        if isinstance(poses, Path):
            for i, pose in enumerate(poses.poses):
                self.set_pose(pose.pose, index=i)
        else:
            for i, pose in enumerate(poses):
                self.set_pose(pose, index=i)


    def set_scale(self, scale):
        """ALL_MARKERS. Set the "scale" of each marker.

        Args:
            scale: can be eighter a tuple(float, float, float) or a single scalar, in which case all 3 components will take it's value.
        """
        if isinstance(scale, float):
            scale = [scale, scale, scale]
        for marker in self.markers:
            marker.scale.x = float(scale[0])
            marker.scale.y = float(scale[1])
            marker.scale.z = float(scale[2])


    def set_type(self, type):
        """ALL_MARKERS. Set the "type" of each marker.

        Args:
            type: one type from Marker.(UPPER_CASE_ATTRIBUT)
        """
        for marker in self.markers:
            marker.type = type
    

    def set_frame_id(self, frame_id):
        """ALL_MARKERS. Set the "frame_id" of each marker.

        Args:
            frame_id: the frame_id of the markers.
        """
        for marker in self.markers:
            marker.header.frame_id = frame_id


    def get_marker_array(self):
        """Returns the Marker array, for publication.

        Returns:
            MarkerArray: Array of markers that compose the item.
        """
        return self.markers

        

class Polyline(Item):
    
    def __init__(self, poses, size=presets.LINE_MEDIUM, color=None):
        if not shared_variables.visualization_enabled: return
        
        super().__init__(1)
        self.set_type(Marker.LINE_STRIP)
        self.set_scale([size, 0., 0.])
        self.set_points(poses)
        self.set_color(color)


class OrientedCube(Item):
        
    def __init__(self, pose, size, color=None):
        if not shared_variables.visualization_enabled: return
        
        super().__init__(1)
        self.set_type(Marker.CUBE)
        self.set_scale(size)
        self.set_pose(pose)
        self.set_color(color)


class Sphere(Item):
            
    def __init__(self, pose, size, color=None):
        if not shared_variables.visualization_enabled: return
        
        super().__init__(1)
        self.set_type(Marker.SPHERE)
        self.set_scale(size)
        self.set_pose(pose)
        self.set_color(color)


class Arrow(Item):
            
    def __init__(self, pose, size=0.5, color=None):
        if not shared_variables.visualization_enabled: return

        super().__init__(1)
        self.set_type(Marker.ARROW)
        self.set_scale([size, size/5., size/5.])
        self.set_pose(pose)
        self.set_color(color)


class Arrows(Item):
    
    def __init__(self, poses, size=0.5, color=None):
        if not shared_variables.visualization_enabled: return

        super().__init__(len(poses))
        for pose in poses:
            arrow = Arrow(pose, size, color)
            self.markers.extend(arrow.get_marker_array())



class Spheres(Item):
        
    def __init__(self, poses, size=0.2, color=None):
        if not shared_variables.visualization_enabled: return

        super().__init__(1)
        self.set_type(Marker.SPHERE_LIST)
        self.set_scale(size)
        self.set_points(poses)
        self.set_color(color)


class Cubes(Item):
            
    def __init__(self, poses, size=0.2, color=None):
        if not shared_variables.visualization_enabled: return

        super().__init__(1)
        self.set_type(Marker.CUBE_LIST)
        self.set_scale(size)
        self.set_points(poses)
        self.set_color(color)


class OrientedCubes(Item):
    
    def __init__(self, poses, size=0.2, color=None):
        if not shared_variables.visualization_enabled: return

        super().__init__(len(poses))
        for pose in poses:
            cube = OrientedCube(pose, size, color)
            self.markers.extend(cube.get_marker_array())


class Points(Item):
            
    def __init__(self, poses, size=presets.POINT_MEDIUM, color=None):
        if not shared_variables.visualization_enabled: return

        super().__init__(1)
        self.set_type(Marker.POINTS)
        self.set_scale(size)
        self.set_points(poses)
        self.set_color(color)