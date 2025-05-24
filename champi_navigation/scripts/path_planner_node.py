#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from champi_interfaces.action import Navigate
from champi_interfaces.msg import CtrlGoal
from geometry_msgs.msg import Pose, PoseStamped

from math import hypot
import numpy as np
import time
from threading import Lock


from champi_navigation.path_planner import PathPlanner, ComputePathResult
from champi_navigation.planning_feedback import ComputePathResult, get_feedback_msg
import champi_navigation.goal_checker as goal_checker
from champi_libraries_py.utils.diagnostics import ExecTimeMeasurer
from champi_libraries_py.utils.timeout import Timeout
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.utils.angles import get_yaw

import diagnostic_msgs
import diagnostic_updater



from rclpy.logging import get_logger

from icecream import ic


class PlannerNode(Node):
    """
    Path planner node. It has an action server. It receives Navigate goals and computes a path to reach them.
    Then it publishes the command to the controller node as a CtrlGoal.
    """

    def __init__(self):
        super().__init__('planner_node')
        #self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Parameters
        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value
        self.waypoint_tolerance = self.declare_parameter('waypoint_tolerance', rclpy.Parameter.Type.DOUBLE).value
        self.debug =self.declare_parameter('debug', rclpy.Parameter.Type.BOOL).value

        # Print parameters
        self.get_logger().info('Path Planner started with the following parameters:')
        self.get_logger().info(f'loop_period: {self.loop_period}')
        self.get_logger().info(f'waypoint_tolerance: {self.waypoint_tolerance}')
        self.get_logger().info(f'debug: {self.debug}')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)

        # Publisher
        self.champi_path_pub = self.create_publisher(CtrlGoal, '/ctrl_goal', 10)
        self.path_publisher_viz = self.create_publisher(Path, '/plan_viz', 10)
        if self.debug: #  debug occupancy grids publishers
            self.debug_pub_raw_path = self.create_publisher(OccupancyGrid, '/raw_path_costmap_viz', 10)
            self.debug_pub_optimized_path = self.create_publisher(OccupancyGrid, '/optimized_path_costmap_viz', 10)

        # Action Server /navigate
        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.navigate_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Path Planner started /navigate server')


        # Path planner object
        self.path_planner = PathPlanner()

        # Timeout object to abort the Navigate goal if it takes too long (robot stuck, goal occupied...)
        self.timeout = Timeout()

        # Diagnostic updater
        updater = diagnostic_updater.Updater(self)
        updater.setHardwareID('none')

        # Diagnostic for execution time of control loop
        self.exec_time_measurer = ExecTimeMeasurer()
        updater.add('Loop exec time', self.exec_time_measurer.produce_diagnostics)

        # Diagnostic for timeout
        updater.add('Timeout', self.timeout.produce_diagnostics) 

        # Diagnostic for PathPlanner
        updater.add('PathPlanner', self.path_planner.produce_diagnostics)

        # Data retreived from topics
        self.robot_pose: Pose = None
        self.costmap = None # Numpy 2D array

        # Current goal handle, that we get when a new goal is received by the action server
        self.goal_handle_navigate = None
        
        # True when we have a goal to reach. Come back to False when the goal is reached or cancelled
        self.planning = False

        # This mutex is used to avoid the execute_callback to be running multiple times concurentlly, which leads to very annoying things.
        # For example, without mutex, when we cancel a goal, the new execute_callback is called althrough the previous one is still running,
        # and they modify the same attributes...
        self.mutex_exec = Lock()


    # ==================================== ROS2 topics Callbacks ==========================================


    def odom_callback(self, msg):
        self.robot_pose = Pose()
        self.robot_pose.position = msg.pose.pose.position
        self.robot_pose.orientation = msg.pose.pose.orientation
    

    def costmap_callback(self, msg):
        # First time we receive the costmap: create the path planner using costmap metadata
        if not self.path_planner.is_initialized:
            self.path_planner.initialize(msg.info.width, msg.info.height, msg.info.resolution)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    

    # ==================================== Action Server Callbacks ==========================================


    def navigate_callback(self, navigate_goal: Navigate.Goal):
        """ Called when a new Navigate goal is received
        """
        self.get_logger().info('New Navigate request received to pose: '
                                f'({navigate_goal.pose.position.x}, {navigate_goal.pose.position.y}, {get_yaw(navigate_goal.pose)*180.0/3.14159})')

        # Store the goal
        self.current_navigate_goal = navigate_goal

        # Cancel the current goal if there is one
        if self.planning:
            self.goal_handle_navigate.abort()

            self.get_logger().info('Received a new Navigate request, cancelling the current one!')
        
        self.planning = True

        return GoalResponse.ACCEPT
    

    def cancel_callback(self, goal_handle):
        """ Called when the goal is cancelled by the client
        """
        self.get_logger().debug('Goal cancelled!')
        if self.planning:
            self.goal_handle_navigate.abort()
        else:
            self.get_logger().warn('No goal to cancel!')  # Can happen if the robot reach the end at the same time as the goal is cancelled
        self.planning = False
        return CancelResponse.ACCEPT

    
    async def execute_callback(self, goal_handle):
        """ Called right after we return GoalResponse.ACCEPT in navigate_callback
        """


        self.mutex_exec.acquire()


        self.goal_handle_navigate = goal_handle


        # =================================== INITIALIZATION ==========================================

        # We're still waiting for first messages (init)
        while rclpy.ok() and (self.robot_pose is None or self.costmap is None) and goal_handle.is_active:
            self.get_logger().info(f'Waiting for init messages, robot_pose={self.robot_pose is not None}, costmap={self.costmap is not None}', throttle_duration_sec=1.)
            
            # Feedback
            feedback_msg = get_feedback_msg(ComputePathResult.INITIALIZING, [], 0)
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(self.loop_period)


        # =================================== MAIN LOOP ==========================================

        navigate_goal_reached = False
        self.timeout.start(self.current_navigate_goal.timeout)

        while rclpy.ok() and goal_handle.is_active and not navigate_goal_reached:

            # Exectution time measurement for diagnostics. Check class definition for details.
            self.exec_time_measurer.start()

            t_loop_start = time.time()

            # Check if the timeout is reached
            if self.timeout.is_elapsed():
                self.get_logger().warn('Timeout reached!')
                goal_handle.abort()
                self.timeout.reset()
                continue

            # Check if goal is reached
            if goal_checker.is_goal_reached(Pose2D(pose=self.current_navigate_goal.pose),
                                            Pose2D(pose=self.robot_pose),
                                            self.current_navigate_goal.end_speed == 0,
                                            self.current_navigate_goal.do_look_at_point,
                                            Pose2D(point=self.current_navigate_goal.look_at_point),
                                            self.current_navigate_goal.robot_angle_when_looking_at_point,
                                            self.current_navigate_goal.linear_tolerance,
                                            self.current_navigate_goal.angular_tolerance):
                navigate_goal_reached = True
                continue

            # Compute path
            path, result = self.path_planner.compute_path(self.robot_pose, self.current_navigate_goal.pose, self.costmap)
            
            
            # We got a valid path, create a CtrlGoal and publish it (+ debug visualizations)
            if path is not None:

                is_waypoint = (result == ComputePathResult.SUCCESS_AVOIDANCE)

                # Create a CtrlGoal
                ctrl_goal = self.create_ctrl_goal_from_navigate_goal(self.current_navigate_goal, is_waypoint)
                ctrl_goal.pose = path[1]

                # Publish goal
                self.champi_path_pub.publish(ctrl_goal)

                # Publish path for visualization
                self.publish_path(path)                    

                if self.debug:
                    costmap_raw_path = self.path_planner.get_raw_path_as_occupancy_grid()
                    costmap_optimized_path = self.path_planner.get_optimized_path_as_occupancy_grid()

                    self.debug_pub_raw_path.publish(costmap_raw_path)
                    self.debug_pub_optimized_path.publish(costmap_optimized_path)


            # Publish action feedback
            feedback_msg = get_feedback_msg(result, path, self.current_navigate_goal.max_linear_speed)
            goal_handle.publish_feedback(feedback_msg)

            self.exec_time_measurer.stop()

            # Sleep to respect the loop period
            sleep_time = max(0, self.loop_period - (time.time() - t_loop_start))
            time.sleep(sleep_time)

        self.planning = False
        self.timeout.reset()
        

        # ============================ FILL ACTION RESULT ====================================

        # stop robot and clear path
        self.publish_stop()
        self.publish_path([])

        result = None
        
        # Check if the goal was aborted
        if not goal_handle.is_active:
            self.get_logger().debug('Action execution stopped because goal was aborted!!')
            result =  Navigate.Result(success=False, message='Goal aborted!')

        # Check if the node is shutting down
        elif not rclpy.ok():
            result = Navigate.Result(success=False, message='Node shutdown!')

        # The goal was reached
        else:
            goal_handle.succeed()
            self.get_logger().debug('Navigate Goal reached!')
            result = Navigate.Result(success=True, message='Goal reached!')
        
        self.mutex_exec.release()
        
        return result


    # ====================================== Utils ==========================================


    def publish_stop(self):
        """Publish a stop command to the controller node. Made by publishing an empty CtrlGoal; as max speed is 0, the robot will stop.
        """
        ctrl_goal = CtrlGoal()
        self.champi_path_pub.publish(ctrl_goal)


    def publish_path(self, path: list[Pose]):
        """Publish a path (for visualization)

        Args:
            path (list[Pose]): The path to publish
        """

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for pose in path:
            path_msg.poses.append(self.pose_to_pose_stamped(pose, 'odom'))

        self.path_publisher_viz.publish(path_msg)


    def pose_to_pose_stamped(self, pose: Pose, frame_id: str) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = pose
        return pose_stamped



    def create_ctrl_goal_from_navigate_goal(self, navigate_goal: Navigate.Goal, is_waypoint) -> CtrlGoal:

        """
        Create a CtrlGoal from a Navigate goal, transferring all the fields.
        Then, you can edit what you need afterwards.
        """

        ctrl_goal = CtrlGoal()

        ctrl_goal.pose = navigate_goal.pose

        if is_waypoint:
            ctrl_goal.end_speed = navigate_goal.max_linear_speed
            ctrl_goal.linear_tolerance = self.waypoint_tolerance
        else:
            ctrl_goal.end_speed = navigate_goal.end_speed
            ctrl_goal.linear_tolerance = navigate_goal.linear_tolerance

        ctrl_goal.max_linear_speed = navigate_goal.max_linear_speed
        ctrl_goal.max_angular_speed = navigate_goal.max_angular_speed

        ctrl_goal.angular_tolerance = navigate_goal.angular_tolerance

        ctrl_goal.do_look_at_point = navigate_goal.do_look_at_point
        ctrl_goal.look_at_point = navigate_goal.look_at_point
        ctrl_goal.robot_angle_when_looking_at_point = navigate_goal.robot_angle_when_looking_at_point

        return ctrl_goal
    

    # ====================================== Main ==========================================


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    # We use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
