#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid, Odometry
from champi_interfaces.action import Navigate
from champi_interfaces.msg import ChampiPath, ChampiSegment, ChampiPoint
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty

from math import pi
import numpy as np
import time

from champi_navigation.path_planner import AStarPathPlanner, ComputePathResult
from rclpy.logging import get_logger


def pose_to_dumb_champi_point(pose: Pose) -> ChampiPoint:
    """
    dumb because it has no properties except the pose
    """
    champi_point = ChampiPoint()
    champi_point.name = "current_pose"
    champi_point.pose = pose
    return champi_point


class PlannerNode(Node):
    """
    SUBS :
        - odom
        - costmap
    PUB :
        - Path

    ACTION SERVER /navigate

    When action server is called with a goal:
        While goal not reached:
            Compute a path and publish it to /plan which will be followed by the path controller
    

    """

    def __init__(self):
        super().__init__('planner_node')
        get_logger('rclpy').info(f"\tLaunching Path planner NODE...")


        self.champi_path_pub = self.create_publisher(ChampiPath, '/plan', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)
        self.path_finished_sub = self.create_subscription(Empty, '/path_finished', self.path_finished_callback, 10)

        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.path_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        
        self.markers_trajectory_pub = self.create_publisher(MarkerArray, '/trajectories', 10)
        self.goal_handle_navigate = None

        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value

        self.robot_pose: Pose = None
        self.index_of_next_waypoint = 1
        self.asked_from_client_champi_path :ChampiPath = None
        self.costmap = None
        self.path_finished = False

        self.path_planner = None

        self.planning = False
        get_logger('rclpy').info("\tPath planner NODE launched!")

    # ==================================== ROS2 Callbacks ==========================================

    def path_finished_callback(self, msg):
        self.path_finished = True
        self.get_logger().warn("PATH FINISHED")

    def odom_callback(self, msg):
        self.robot_pose = Pose()
        self.robot_pose.position = msg.pose.pose.position
        self.robot_pose.orientation = msg.pose.pose.orientation
    

    def path_callback(self, navigate_goal:Navigate.Goal):
        self.get_logger().info('New path received!')

        champi_path : ChampiPath = navigate_goal.path
        self.asked_from_client_champi_path = champi_path

        # Cancel the current goal if there is one
        if self.planning:
            self.goal_handle_navigate.abort()
            self.get_logger().info('Received a new path, cancelling the current one!')

        self.planning = True
        self.path_finished = False
        return GoalResponse.ACCEPT
    
    def display_segments(self, champi_path: ChampiPath):
        s = ""
        for seg in champi_path.segments:
            seg: ChampiSegment
            s += str(seg.start.pose.position.x)+" "+str(seg.start.pose.position.y) + "--"
            s += str(seg.end.pose.position.x)+" "+str(seg.end.pose.position.y)
            s += "\n"
        return s

    async def execute_callback(self, goal_handle):
        self.goal_handle_navigate = goal_handle

         # We're still waiting for first messages (init)
        while rclpy.ok() and self.planning and (self.robot_pose is None or self.costmap is None):
            self.get_logger().info('Waiting for init messages', throttle_duration_sec=1.)
            # TODO feedback
            time.sleep(self.loop_period)

        while rclpy.ok() and self.planning and not self.path_finished:
            t_loop_start = time.time()
            
            # Compute the path, see Readme.md
            global_champi_path = ChampiPath()
            
            marker_array = MarkerArray()
            for i, s in enumerate(self.asked_from_client_champi_path.segments):
                s:ChampiSegment
                p:ChampiPoint = s.start
                marker_array.markers.append(self.create_marker(p.pose, i))
            marker_array.markers.append(self.create_marker(self.asked_from_client_champi_path.segments[-1].end.pose, len(self.asked_from_client_champi_path.segments)))
                    
            self.markers_trajectory_pub.publish(marker_array)
            

            # modify the first point of the path to set it to current_pose, because it has not been set in robot_navigator
            self.asked_from_client_champi_path.segments[0].start = pose_to_dumb_champi_point(self.robot_pose)

            for champi_segment in self.asked_from_client_champi_path.segments[self.index_of_next_waypoint-1:]:
                # for each segment, we compute the path
                champi_segment:ChampiSegment
                local_champi_path, result = self.path_planner.compute_path(champi_segment.start.pose, champi_segment.end.pose, self.costmap)
                local_champi_path: ChampiPath

                if result != ComputePathResult.SUCCESS:
                    self.get_logger().warn(f'Path computation failed: {result.name}', throttle_duration_sec=0.5)
                    # TODO gérer la situation

                # we set the params of the segment to the local_path
                for seg in local_champi_path.segments:
                    seg:ChampiSegment
                    # seg.header = champi_segment.header
                    seg.do_look_at_point = champi_segment.do_look_at_point
                    seg.look_at_point = champi_segment.look_at_point
                    seg.robot_angle_when_looking_at_point = champi_segment.robot_angle_when_looking_at_point
                    seg.max_linear_speed = champi_segment.max_linear_speed
                    seg.max_angular_speed = champi_segment.max_angular_speed
                    seg.name = champi_segment.name
                # set the params of the first and last point
                local_champi_path.segments[0].start.point_type = champi_segment.start.point_type
                local_champi_path.segments[0].start.linear_tolerance = champi_segment.start.linear_tolerance
                local_champi_path.segments[0].start.angular_tolerance = champi_segment.start.angular_tolerance
                local_champi_path.segments[0].start.robot_should_stop_here = champi_segment.start.robot_should_stop_here

                local_champi_path.segments[-1].end.point_type = champi_segment.end.point_type
                local_champi_path.segments[-1].end.linear_tolerance = champi_segment.end.linear_tolerance
                local_champi_path.segments[-1].end.angular_tolerance = champi_segment.end.angular_tolerance
                local_champi_path.segments[-1].end.robot_should_stop_here = champi_segment.end.robot_should_stop_here
                
                # set the params of the path
                # local_champi_path.header = self.asked_from_client_champi_path.header
                local_champi_path.name = self.asked_from_client_champi_path.name
                local_champi_path.max_time_allowed = self.asked_from_client_champi_path.max_time_allowed
                local_champi_path.forcing_type = self.asked_from_client_champi_path.forcing_type


                # then we concatenate all paths in a global one
                for local_sub_champi_segment in local_champi_path.segments:
                    global_champi_path.segments.append(local_sub_champi_segment)
            # global_champi_path.max_time_allowed = self.asked_from_client_champi_path.max_time_allowed # TODO


            # Publish the path and feedback
            self.champi_path_pub.publish(global_champi_path)

            # TODO feedback

            self.get_logger().info(f'. \t\t loop_exec_time={round(time.time() - t_loop_start, 5)}')
            time.sleep(self.loop_period - (time.time() - t_loop_start))


        # Publish a final empty path
        path = ChampiPath()
        path.header.stamp = self.get_clock().now().to_msg()
        self.champi_path_pub.publish(path)

        result = None

        if not self.planning:
            self.get_logger().info('Action execution stopped because goal was aborted!!')
            result =  Navigate.Result(success=False, message='Goal aborted!')
        else:
            goal_handle.succeed()
            self.get_logger().info('Goal reached!')
            self.get_logger().warn('')
            result = Navigate.Result(success=True, message='Goal reached!')
        
        self.planning = False
        return result


    def costmap_callback(self, msg):
        if self.path_planner is None:
            self.path_planner = AStarPathPlanner(msg.info.width, msg.info.height, msg.info.resolution)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)


    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled!')
        self.goal_handle_navigate.abort()
        self.planning = False
        return CancelResponse.ACCEPT

    # ====================================== Utils ==========================================

    def create_marker(self, pose:Pose, marker_id):
        marker = Marker()
        marker.header.frame_id = "odom"  # Remplacer par le frame approprié
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "trajectory"
        marker.id = marker_id
        marker.type = Marker.CUBE  # Utilisation d'un marqueur sphérique
        marker.action = Marker.ADD

        # Définir la position et l'orientation à partir de Pose
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z

        marker.pose.orientation.x = pose.orientation.x
        marker.pose.orientation.y = pose.orientation.y
        marker.pose.orientation.z = pose.orientation.z
        marker.pose.orientation.w = pose.orientation.w

        # Paramètres de mise à l'échelle et de couleur
        marker.scale.x = 0.05  # Taille du marqueur en x
        marker.scale.y = 0.05  # Taille du marqueur en y
        marker.scale.z = 0.05  # Taille du marqueur en z

        marker.color.a = 1.0  # Opacité (1.0 = opaque, 0.0 = transparent)
        marker.color.r = 0.0  # Rouge
        marker.color.g = 0.0  # Vert
        marker.color.b = 1.0  # Bleu

        return marker


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
