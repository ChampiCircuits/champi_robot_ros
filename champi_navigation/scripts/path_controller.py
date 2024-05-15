#!/usr/bin/env python3

from champi_navigation.utils import Vel, dist_point_to_line, RobotState
from champi_navigation.cmd_vel_updaters import CmdVelUpdaterWPILib


from math import pi, atan2, sqrt
from icecream import ic


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

        
class PoseControl(Node):
    def __init__(self):
        super().__init__('pose_control_node')

        self.control_loop_period = self.declare_parameter('control_loop_period', 0.1).value        

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.current_pose_sub = self.create_subscription(Odometry, '/odometry/filtered', self.current_pose_callback, 10)

        # Timers
        self.timer = self.create_timer(self.control_loop_period, self.control_loop_spin_once)

        # Diagnostic
        self.last_time_ctrl_loop = None

        # Objects instanciation

        self.robot_current_state = None
        self.cmd_vel_updater = CmdVelUpdaterWPILib()

        # Variables related to goals
        
        # Path to follow [[x, y, theta], ...].
        self.cmd_path = []
        # Current goal index in cmd_path
        self.i_goal = None
        # Convenience variable that is equal to cmd_path[i_goal]
        self.current_seg_end = None
        self.current_seg_start = None
        # Convenience variable that is equal to cmd_path[i_goal-1]
        # self.prev_goal = None

        self.goal_reached = False

    def current_pose_callback(self, msg):
        """Callback for the current pose message. It is called when a new pose is received from topic."""
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)]
        vel = Vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        vel = Vel.to_global_frame(pose, vel)
        self.robot_current_state = RobotState(pose, vel)

    def path_callback(self, msg):
        """Callback for the path message. It is called when a new path is received from topic."""

        # Return if we have already received the robot current pose (set_cmd_path needs it)
        if self.robot_current_state is None:
            return

        self.set_cmd_path(msg.poses)
        # s = "RECEIVED PATH : "
        # for p in msg.poses:
        #     x,y = p.pose.position.x, p.pose.position.y
        #     s+=f"({x:.5f}, {y:.5f}) "
        # self.get_logger().info(s)



    def set_cmd_path(self, cmd_path):
        """First pose must be the current robot pose."""
        if len(cmd_path) == 0:
            self.i_goal = None
            self.current_seg_end = None
            self.current_seg_start = None
            return


        # calcul de i_goal
        # on veut savoir où on se trouve sur la trajectoire, donc on regarde la distance entre la position actuelle et chaque segment du path
        segs = []
        for i in range(len(cmd_path)-1):
            p1 = cmd_path[i].pose.position
            p2 = cmd_path[i+1].pose.position
            segs.append([(p1.x, p1.y), (p2.x, p2.y)])

        min_dist = 100000000000000000
        min_i = 0
        for seg in segs:
            # si dist point to line
            if seg[0] != seg[1]: # division par 0
                if dist_point_to_line(self.robot_current_state.pose[:2], [seg[0], seg[1]]) < min_dist:
                    min_dist = dist_point_to_line(self.robot_current_state.pose[:2], [seg[0], seg[1]])
                    min_i = segs.index(seg)

        self.i_goal = min_i+1

        self.current_seg_end = self.pose_stamped_to_array(cmd_path[self.i_goal])

        if self.cmd_path == []: # This is the first path we receive
            self.current_seg_start = self.pose_stamped_to_array(cmd_path[0])

        elif self.is_this_a_new_path(cmd_path): # This is a new path so we need to set the start of the current segment
            self.current_seg_start = self.pose_stamped_to_array(cmd_path[self.i_goal-1])

        # Else, we keep the current start of the segment

        self.cmd_path = cmd_path


    def is_this_a_new_path(self, cmd_path):
        # The path is simply an updated version of the previous path if the second points are (almost) the same

        return not self.are_points_close(cmd_path[1], self.cmd_path[1])


    def are_points_close(self, p1, p2):
        return abs(p1.pose.position.x - p2.pose.position.x) < 0.1 and abs(p1.pose.position.y - p2.pose.position.y) < 0.1

    def pose_stamped_to_array(self, pose_stamped):
        return [pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                2*atan2(pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)]

    def control_loop_spin_once(self):

        if self.robot_current_state is None:
            return

        if self.i_goal is None:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.
            cmd_vel.linear.y = 0.
            cmd_vel.angular.z = 0.

            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        goal_reached = self.is_current_goal_reached()

        if goal_reached and not self.goal_reached:
            self.goal_reached = True
            self.switch_to_next_goal()

        elif not goal_reached and self.goal_reached:
            self.goal_reached = False

        # We reached the last pose of the cmd path
        if self.i_goal is None:
            return Vel(0., 0., 0.).to_twist()


        # if it's not the last goal: arrival speed is 0.3 m/s
        arrival_speed = 0.3
        if self.i_goal == len(self.cmd_path)-1:
            arrival_speed = 0.


        # Compute the command velocity
        arrival_angle = 2*atan2(self.cmd_path[-1].pose.orientation.z, self.cmd_path[-1].pose.orientation.w)

        cmd_vel = self.cmd_vel_updater.compute_cmd_vel(self.robot_current_state,
                                                       self.current_seg_start,
                                                       self.current_seg_end,
                                                       arrival_speed,
                                                       arrival_angle)
        # express in the base_link frame
        cmd_vel = Vel.to_robot_frame(self.robot_current_state.pose, cmd_vel)
        # convert to cmd_twist
        cmd_twist = cmd_vel.to_twist()

        # Publish the command
        # ic(cmd_twist)
        self.cmd_vel_pub.publish(cmd_twist)



    def is_current_goal_reached(self):
        """Checks if the goal is reached and switch to the next one if it is the case.
        Should not be called if i_goal is None = no path to follow."""

        error_max_lin = 0.05
        error_max_ang = 0.05

        if abs(self.robot_current_state.pose[0] - self.current_seg_end[0]) < error_max_lin and abs(self.robot_current_state.pose[1] - self.current_seg_end[1]) < error_max_lin:
            if self.check_angle(self.robot_current_state.pose[2], self.current_seg_end[2], error_max_ang):
                return True
    
        return False


    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max


    def switch_to_next_goal(self):
        """If the current goal wasn't the last one, switch to the next one by incrementing i_goal.
        Otherwise, clear the cmd_path and set i_goal to None."""

        self.i_goal += 1

        if self.i_goal == len(self.cmd_path):
            self.cmd_path = []
            self.i_goal = None
            self.current_seg_start = None
            self.current_seg_end = None
            # self.prev_goal = None
            return

        self.current_seg_start = self.current_seg_end
        self.current_seg_end = self.pose_stamped_to_array(self.cmd_path[self.i_goal])


def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PoseControl()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()