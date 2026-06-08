from rclpy.node import Node

from champi_interfaces.msg import CtrlGoal
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from champi_interfaces.action import Navigate


class PoseControllerManager:
    def __init__(self, node: Node, waypoint_speed_linear: float, waypoint_tolerance: float):
        self.node = node
        self.waypoint_speed_linear = waypoint_speed_linear
        self.waypoint_tolerance = waypoint_tolerance

        self.ctrl_goal_pub = self.node.create_publisher(CtrlGoal, '/ctrl_goal', 10)

    def publish_stop(self):
        ctrl_goal = CtrlGoal()
        self.ctrl_goal_pub.publish(ctrl_goal)


    def _create_ctrl_goal_from_navigate_goal(self, navigate_goal: Navigate.Goal, is_waypoint) -> CtrlGoal:
        ctrl_goal = CtrlGoal()
        ctrl_goal.pose = navigate_goal.pose

        if is_waypoint:
            ctrl_goal.end_speed = self.waypoint_speed_linear
            ctrl_goal.linear_tolerance = self.waypoint_tolerance
            ctrl_goal.max_linear_speed = self.waypoint_speed_linear
            ctrl_goal.do_look_at_point = False
            ctrl_goal.look_at_point = Point()
            ctrl_goal.robot_angle_when_looking_at_point = 0.0
        else:
            ctrl_goal.end_speed = navigate_goal.end_speed
            ctrl_goal.linear_tolerance = navigate_goal.linear_tolerance
            ctrl_goal.max_linear_speed = navigate_goal.max_linear_speed

        ctrl_goal.max_angular_speed = navigate_goal.max_angular_speed
        ctrl_goal.accel_linear = navigate_goal.accel_linear
        ctrl_goal.accel_angular = navigate_goal.accel_angular
        ctrl_goal.angular_tolerance = navigate_goal.angular_tolerance
        if not is_waypoint:
            ctrl_goal.do_look_at_point = navigate_goal.do_look_at_point
            ctrl_goal.look_at_point = navigate_goal.look_at_point
            ctrl_goal.robot_angle_when_looking_at_point = navigate_goal.robot_angle_when_looking_at_point

        return ctrl_goal
    
    def publish_ctrl_goal(self, goal, metadata: Navigate.Goal, is_waypoint):
        ctrl_goal = self._create_ctrl_goal_from_navigate_goal(metadata, is_waypoint=is_waypoint)
        ctrl_goal.pose = goal.to_ros_pose()
        self.ctrl_goal_pub.publish(ctrl_goal)
