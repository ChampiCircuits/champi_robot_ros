#!/usr/bin/env python3

from champi_navigation.utils import Vel
from champi_navigation.pid import PID

from math import pi, atan2, sqrt
import matplotlib.pyplot as plt
from icecream import ic
from enum import Enum
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

# class CmdVelUpdater:
#     def __init__(self):
#         # PIDs
#         self.pid_pos_x = PID(1, 0, 0)
#         self.pid_pos_y = PID(1, 0, 0)
#         self.pid_pos_theta = PID(1, 0, 0,)


#     def compute_cmd_vel(self, robot_state, pose_goal, dt):
#         # if it's shorter to turn in the other direction, we do it
#         # TODO not always working
#         error_theta = pose_goal[2] - robot_state.current_pose[2]
#         if abs(error_theta) > pi:
#             if error_theta > 0:
#                 error_theta -= 2*pi
#             else:
#                 error_theta += 2*pi
        
#         cmd_vel = Vel()

#         cmd_vel.theta = self.pid_pos_theta.update(error_theta, dt)

#         # # PID
#         cmd_vel.x = self.pid_pos_x.update(pose_goal[0] - robot_state.current_pose[0], dt)
#         cmd_vel.y = self.pid_pos_y.update(pose_goal[1] - robot_state.current_pose[1], dt)

#         return cmd_vel

class CmdVelUpdater2:
    def __init__(self):
        self.start_pose = None
        self.goal_pose = None
        
        self.vel_profile_mag = TrapezoidalVelocityProfile(0.5, 0.5)
        self.vel_profile_theta = TrapezoidalVelocityProfile(2.0, 1.0)

        self.pid_correct_dir = PID(2, 0, 1)
        self.last_time_called = time.time()
    
    def compute_cmd_vel(self, robot_current_pose, pose_goal):
        dt = time.time() - self.last_time_called
        self.last_time_called = time.time()

        # Quick fix parce qu'il faut avoir la pose de depart à vitesse = 0
        if self.goal_pose != pose_goal:

            self.start_pose = robot_current_pose
            self.goal_pose = pose_goal
            print("start_pose",self.start_pose)
            print("goal_pose",self.goal_pose)

            distance_start_to_goal = ((pose_goal[0] - self.start_pose[0])**2 + (pose_goal[1] - self.start_pose[1])**2)**0.5
            # distance_start_to_goal = ((pose_goal[0] - self.start_pose.pose.position.x)**2 + (pose_goal[1] - self.start_pose.pose.position.y)**2)**0.5
            self.vel_profile_mag.set_new_goal(distance_start_to_goal, 0)

            # if it's shorter to turn in the other direction, we do it
            error_theta = pose_goal[2] - robot_current_pose[2]
            if abs(error_theta) > pi:
                if error_theta > 0:
                    error_theta -= 2*pi
                else:
                    error_theta += 2*pi

            self.vel_profile_theta.set_new_goal(error_theta, 0)

        # Compute distance to goal
        # distance_to_goal = ((pose_goal[0] - robot_current_pose[0])**2 + (pose_goal[1] - robot_current_pose[1])**2)**0.5
        
        mag = self.vel_profile_mag.compute_vel(None)
        theta = self.vel_profile_theta.compute_vel(None)

        angle_vec_dir = atan2(pose_goal[1] - robot_current_pose[1], pose_goal[0] - robot_current_pose[0])


        dist=0
        if robot_current_pose[:2] != self.start_pose[:2]: # sinon div par 0
            dist = dist_point_to_line_signed(robot_current_pose[:2], [self.start_pose[:2], self.goal_pose[:2]])
        
        correction = self.pid_correct_dir.update(dist, dt)

        correction_max = 0.5
        if correction > correction_max:
            correction = correction_max
        elif correction < -correction_max:
            correction = -correction_max
        
        angle_vec_dir += correction

        cmd_vel = Vel()
        cmd_vel.init_from_mag_ang(mag, angle_vec_dir, theta)

        return cmd_vel

    
def dist_point_to_line (point, line):
    x0, y0 = point
    x1, y1 = line[0]
    x2, y2 = line[1]
    return abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)

def dist_point_to_line_signed ( point, line):
    x0, y0 = point
    x1, y1 = line[0]
    x2, y2 = line[1]
    return ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)


class TrapezoidalVelocityProfile:

    class ProfileState(Enum):
        ACCELERATION = 1
        FLAT = 2
        DECELERATION = 3


    def __init__(self, max_speed, max_acceleration):
        self.v_max = max_speed
        self.a_max = max_acceleration

        self.start_pos = None
        self.end_pos = None

        self.state = None

        self.t_start = None

        self.t_end_acc = None
        self.t_end_flat = None
        self.t_end_dec = None

        self.error_negative = False

    
    def set_new_goal(self, start_pos, goal_pos):

        self.start_pos = start_pos
        self.end_pos = goal_pos

        if self.start_pos > self.end_pos:
            self.error_negative = True

        tf = 2 * sqrt(abs(self.end_pos - self.start_pos) / self.a_max)

        midpoint_vel = self.a_max * tf / 2

        if midpoint_vel <= self.v_max: # 2 segments case
            self.t_end_acc = tf/2
            self.t_end_flat = None
            self.t_end_dec = tf
        
        else: # 3 segments case
            tf = abs(self.end_pos - self.start_pos) / self.v_max + self.v_max / self.a_max
            self.t_end_acc = self.v_max / self.a_max
            self.t_end_flat = tf - self.t_end_acc
            self.t_end_dec = tf

        self.t_start = time.time()

    
    def compute_vel(self, pos_current):

        t = time.time() - self.t_start

        # Update profile state
        if t < self.t_end_acc:
            self.state = self.ProfileState.ACCELERATION
        elif self.t_end_flat is not None and t < self.t_end_flat:
            self.state = self.ProfileState.FLAT
        elif t < self.t_end_dec:
            self.state = self.ProfileState.DECELERATION
        else:
            return 0

        # Update cmd
        cmd = 0
        if self.state == self.ProfileState.ACCELERATION:
            cmd = self.a_max * t
        elif self.state == self.ProfileState.FLAT:
            cmd = self.v_max
        else:
            if self.t_end_flat is not None:
                cmd = self.v_max - self.a_max * (t - self.t_end_flat)
            else:
                midpoint_vel = self.a_max * self.t_end_dec / 2
                cmd = midpoint_vel - self.a_max * (t - self.t_end_acc)
        
        if not self.error_negative:
            cmd = -cmd

        # ic(self.state, t, self.t_end_acc, self.t_end_flat, self.t_end_dec, cmd)

        return cmd




        
class PoseControl(Node):
    def __init__(self):
        super().__init__('pose_control_node')
        self.control_loop_period = self.declare_parameter('control_loop_period', 0.1).value        

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/cmd_path', self.path_callback, 10)
        self.current_pose_sub = self.create_subscription(Odometry, '/odom', self.current_pose_callback, 10)

        # Timers
        self.timer = self.create_timer(self.control_loop_period, self.control_loop_spin_once)

        # Diagnostic
        self.last_time_ctrl_loop = None

        # Objects instanciation

        self.robot_current_pose = None # TODO s'en debarasser ?
        self.cmd_vel_updater = CmdVelUpdater2()

        # Variables related to goals
        
        # Path to follow [[x, y, theta], ...]. The first pose must be the current robot pose.
        self.cmd_path = []
        # Current goal index in cmd_path
        self.i_goal = None
        # Convenience variable that is equal to cmd_path[i_goal]
        self.current_goal = None
        # Convenience variable that is equal to cmd_path[i_goal-1]
        # self.prev_goal = None

    def current_pose_callback(self, msg):
        """Callback for the current pose message. It is called when a new pose is received from topic."""
        self.robot_current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)]
        # ic("pc: RECEIVED CURRENT POSE :")
        # ic(self.robot_current_pose)

    def path_callback(self, msg):
        """Callback for the path message. It is called when a new path is received from topic."""
        self.set_cmd_path(msg.poses)
        # ic()
        # ic("pc: RECEIVED PATH :")
        s=""
        for p in msg.poses:
            x,y = p.pose.position.x, p.pose.position.y
            s+=f"({x:.5f}, {y:.5f}) "
        # print(s)
        # ic()


    def set_cmd_path(self, cmd_path):
        """First pose must be the current robot pose."""
        if len(cmd_path) == 0:
            self.i_goal = None
            self.current_goal = None
            # self.prev_goal = None
        else:
            if self.i_goal == None:
                self.i_goal = 0
            self.cmd_path = cmd_path

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
                    if dist_point_to_line(self.robot_current_pose[:2], [seg[0], seg[1]]) < min_dist:
                        min_dist = dist_point_to_line(self.robot_current_pose[:2], [seg[0], seg[1]])
                        min_i = segs.index(seg)
            # print("current i_goal : ",self.i_goal)

            if min_i+1>self.i_goal: #quickfix TODO
                self.i_goal = min_i+1
            # print("\t\t nearest seg : ",min_i,"donc i_goal = ",self.i_goal)
            # convert the cmd_path[self.i_goal] to [x, y, theta]
            self.current_goal = [cmd_path[self.i_goal].pose.position.x, 
                                 cmd_path[self.i_goal].pose.position.y, 
                                 2*atan2(cmd_path[1].pose.orientation.z, cmd_path[self.i_goal].pose.orientation.w)]
            # self.prev_goal = self.cmd_path[0]    

    def control_loop_spin_once(self):
        if self.i_goal is None:
            return Vel(0., 0., 0.).to_twist() # No cmd path to follow, return
        
        goal_reached = self.is_current_goal_reached()
        if goal_reached:
            self.switch_to_next_goal()
        # print("i_goal:", self.i_goal)
        
        # We reached the last pose of the cmd path
        if self.i_goal is None:
            return Vel(0., 0., 0.).to_twist()

        # Compute the command velocity
        cmd_vel = self.cmd_vel_updater.compute_cmd_vel(self.robot_current_pose, self.current_goal)
        # express in the base_link frame
        cmd_vel = Vel.to_robot_frame(self.robot_current_pose, cmd_vel)    
        # convert to cmd_twist
        cmd_twist = cmd_vel.to_twist()

        

        # publish the velocity
        cmd_twist_stamped = TwistStamped()
        cmd_twist_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd_twist_stamped.header.frame_id = "base_link"
        cmd_twist_stamped.twist = cmd_twist

        # Publish the command
        self.cmd_vel_pub.publish(cmd_twist_stamped) #TODO remettre

        return


    def is_current_goal_reached(self):
        """Checks if the goal is reached and switch to the next one if it is the case.
        Should not be called if i_goal is None = no path to follow."""

        error_max_lin = 0.001
        error_max_ang = 0.01

        # ic(self.robot_current_pose)
        # ic(self.current_goal)
        if abs(self.robot_current_pose[0] - self.current_goal[0]) < error_max_lin and abs(self.robot_current_pose[1] - self.current_goal[1]) < error_max_lin:
            if self.check_angle(self.robot_current_pose[2], self.current_goal[2], error_max_ang):
                print("CURRENT GOAL REACHED")
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
            self.current_goal = None
            # self.prev_goal = None
            return
        
        self.current_goal = [self.cmd_path[self.i_goal].pose.position.x, 
                             self.cmd_path[self.i_goal].pose.position.y, 
                             2*atan2(self.cmd_path[self.i_goal].pose.orientation.z, self.cmd_path[self.i_goal].pose.orientation.w)]

        # self.prev_goal = self.cmd_path[self.i_goal-1]
        

def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PoseControl()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()