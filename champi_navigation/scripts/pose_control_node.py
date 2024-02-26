#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import geometry_msgs.msg

from champi_navigation.kinematic_models import Robot_Kinematic_Model, Obstacle_static_model, Table_static_model
import champi_navigation.avoidance as avoidance
import champi_navigation.gui as gui
from champi_navigation.pid import PID

from champi_navigation.gui_v2 import GuiV2

from icecream import ic
from math import pi, atan2, cos, sin
from shapely import Point

WIDTH, HEIGHT = 900, 600  # window
TABLE_WIDTH, TABLE_HEIGHT = 3, 2  # Table size in m
OFFSET = 0.15 # TODO rayon du self.robot, à voir Etienne

class PoseControl(Node):

    def __init__(self):
        super().__init__('pose_control_node')

        # Parameters
        self.control_loop_period = self.declare_parameter('control_loop_period', 0.1).value
        self.viz = self.declare_parameter('viz', True).value

        # Objects instanciation
        self.robot = Robot_Kinematic_Model()
        self.obstacle = Obstacle_static_model(center_x=1, center_y= 1, width= 0.1, height= 0.1,offset=OFFSET)
        self.table = Table_static_model(TABLE_WIDTH, TABLE_HEIGHT, offset=OFFSET)
        # self.gui = gui.Gui(self.robot, self.obstacle, self.table)

        self.gui_v2 = GuiV2(self)

        # Go to goal v1
        self.pid_pos_x = PID(1, 0, 0, self.control_loop_period)
        self.pid_pos_y = PID(1, 0, 0, self.control_loop_period)
        self.pid_pos_theta = PID(1, 0, 0, self.control_loop_period)
        self.delta_t = self.control_loop_period  # Time between two updates


        # Variables
        self.latest_goal = None # Set by the goal_callback when new goal is received

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.markers_pub_obstacle = self.create_publisher(Marker, '/obstacle_marker', 10)
        self.markers_pub_obstacle_offset = self.create_publisher(Marker, '/obstacle_offset_marker', 10)
        self.markers_pub_path = self.create_publisher(Marker, '/path_markers', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_sub  # prevent unused variable warning
        
        # Timers
        self.timer = self.create_timer(self.control_loop_period, self.control_loop_callback)

        # TF related
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def goal_callback(self, msg):
        self.latest_goal = msg

    def control_loop_callback(self):
        self.update()

    def update_robot_pose_from_tf(self):
        t = None
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time())
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"odom"} to {"base_link"}: {ex}')
            return
        
        q = t.transform.rotation
        self.robot.pos = [t.transform.translation.x, t.transform.translation.y, 2*atan2(q.z, q.w)]

        # Conversion entre -pi et pi
        if self.robot.pos[2] > pi:
            self.robot.pos[2] -= 2*pi
        elif self.robot.pos[2] < -pi:
            self.robot.pos[2] += 2*pi


    def check_goal_reached(self):
        error_max_lin = 1
        error_max_ang = 0.01

        if (len(self.robot.goals_positions))==0:
            return True

        self.robot.current_goal = self.robot.goals_positions[0]

        if abs(self.robot.pos[0] - self.robot.current_goal[0]) < error_max_lin and abs(self.robot.pos[1] - self.robot.current_goal[1]) < error_max_lin:
            if self.check_angle(self.robot.pos[2], self.robot.current_goal[2], error_max_ang):
                self.goto_next_goal()
                ic("GOAL REACHED")

                self.robot.goal_reached = True

    def goto(self, x, y, theta):
        # if it's shorter to turn in the other direction, we do it
        # TODO not always working
        error_theta = theta - self.robot.pos[2]
        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta -= 2*pi
            else:
                error_theta += 2*pi

        self.robot.angular_speed = self.pid_pos_theta.update(error_theta)

        # # PID
        self.robot.linear_speed = [
            self.pid_pos_x.update(x - self.robot.pos[0]),
            self.pid_pos_y.update(y - self.robot.pos[1])
        ]

    def goto_next_goal(self):
        # called when a goal is reached to set the new current goal
        self.robot.goal_reached = False
        self.robot.has_finished_rotation = False
        if len(self.robot.goals_positions)>0:
            self.robot.goals_positions.pop(0)
        if len(self.robot.goals_positions)>0:
            self.robot.current_goal = self.robot.goals_positions[0]
        

    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max


    def recompute_path(self, obstacle, table, goal_pos=None):
        if goal_pos is None:
            if len(self.robot.goals_positions) > 0:
                theta = self.robot.goals_positions[-1][2] # use the theta of the goal for each point
                goal = Point(self.robot.goals_positions[-1][0],self.robot.goals_positions[-1][1])
            else:
                return
        else:
            theta = goal_pos[2]
            goal = Point(goal_pos[0],goal_pos[1])

        start = Point(self.robot.pos[0],self.robot.pos[1])
        self.robot.graph, self.robot.dico_all_points = avoidance.create_graph(start, goal, obstacle.expanded_obstacle_poly, table.expanded_poly)
        path = avoidance.find_avoidance_path(self.robot.graph, 0, 1)
        if path is not None:
            self.robot.path_nodes = path.nodes # mais en soit renvoie aussi le coût
            goals = []

            for p in self.robot.path_nodes[1:]: # we don't add the start point
                goals.append([float(self.robot.dico_all_points[p][0]),float(self.robot.dico_all_points[p][1]), theta])
            self.robot.goals_positions = goals
    

    def update_goal(self):
        if self.latest_goal is not None:
            x = self.latest_goal.pose.position.x
            y = self.latest_goal.pose.position.y
            q = self.latest_goal.pose.orientation
            theta = 2*atan2(q.z, q.w)
            self.robot.goals_positions.append([x, y, theta])
            self.latest_goal = None
        
        # if self.gui.pose_request is not None:
        #     self.robot.goals_positions.append(self.gui.pose_request)
        #     self.gui.pose_request = None

    def update(self):

        self.update_robot_pose_from_tf()
        if self.robot.pos is None: # TF could not be received
            return

        self.update_goal()

        if self.robot.goals_positions is None: # Initialisation
            self.robot.goals_positions = []
            self.robot.goals_positions.append([self.robot.pos[0], self.robot.pos[1], self.robot.pos[2]])

        self.check_goal_reached()

        self.goto(self.robot.current_goal[0],
        self.robot.current_goal[1],
        self.robot.current_goal[2])
                
        self.recompute_path(self.obstacle, self.table)
    
        # publish the velocity (expressed in the base_link frame)
        twist = Twist()
        twist.linear.x = self.robot.linear_speed[0] * cos(self.robot.pos[2]) + self.robot.linear_speed[1] * sin(self.robot.pos[2])
        twist.linear.y = -self.robot.linear_speed[0] * sin(self.robot.pos[2]) + self.robot.linear_speed[1] * cos(self.robot.pos[2])
        twist.angular.z = self.robot.angular_speed
        self.cmd_vel_pub.publish(twist)

        if self.viz:
            # self.gui.update()
            self.gui_v2.draw_goal_poses(self.robot.pos, self.robot.goals_positions)
            self.gui_v2.update()
            self.rviz_draw()


    def rviz_draw(self):

        # OBSTACLE marker
        marker_obstacle = Marker()
        marker_obstacle.header.frame_id = "odom"
        marker_obstacle.type = Marker.CUBE
        marker_obstacle.action = Marker.ADD

        marker_obstacle.scale.x = float(self.obstacle.width)
        marker_obstacle.scale.y = float(self.obstacle.height)
        marker_obstacle.scale.z = 0.3

        marker_obstacle.color.a = 1.0
        marker_obstacle.color.r = 0.5
        marker_obstacle.color.g = 0.0
        marker_obstacle.color.b = 0.0
        marker_obstacle.pose.position.x = float(self.obstacle.center_x)
        marker_obstacle.pose.position.y = float(self.obstacle.center_y)
        marker_obstacle.pose.position.z = 0.0

        self.markers_pub_obstacle.publish(marker_obstacle)

        # OFFSET OBSTACLE MARKER
        marker_obstacle_offset = Marker()
        marker_obstacle_offset.header.frame_id = "odom"
        marker_obstacle_offset.type = Marker.CUBE
        marker_obstacle_offset.action = Marker.ADD

        marker_obstacle_offset.scale.x = float(self.obstacle.width+2*OFFSET)
        marker_obstacle_offset.scale.y = float(self.obstacle.height+2*OFFSET)
        marker_obstacle_offset.scale.z = 0.05

        marker_obstacle_offset.color.a = 1.0
        marker_obstacle_offset.color.r = 1.0
        marker_obstacle_offset.color.g = 1.0
        marker_obstacle_offset.color.b = 0.0
        marker_obstacle_offset.pose.position.x = float(self.obstacle.center_x)
        marker_obstacle_offset.pose.position.y = float(self.obstacle.center_y)
        marker_obstacle_offset.pose.position.z = 0.0

        self.markers_pub_obstacle_offset.publish(marker_obstacle_offset)

        ic(self.robot.goals_positions)

        # path markers
        if self.robot.goals_positions is not None:
            marker_path = Marker(type=Marker.LINE_STRIP, ns='points_and_lines', action=Marker.ADD)
            marker_path.header.frame_id = "odom"
            marker_path.scale.x = 0.05
            marker_path.color.a = 1.0
            marker_path.color.r = 0.0
            marker_path.color.g = 1.0
            marker_path.color.b = 0.0
            marker_path.points = []

            # add current pos if odd
            for i in range(len(self.robot.goals_positions)-1):
                p1 = self.robot.goals_positions[i]
                p2 = self.robot.goals_positions[i+1]
                marker_path.points.append(geometry_msgs.msg.Point(x=float(p1[0]), y=float(p1[1]), z=0.0))
                marker_path.points.append(geometry_msgs.msg.Point(x=float(p2[0]), y=float(p2[1]), z=0.0))
                # si pas le dernier on ajoute encore
                if i != len(self.robot.goals_positions)-2:
                    marker_path.points.append(geometry_msgs.msg.Point(x=float(p2[0]), y=float(p2[1]), z=0.0))

            # publish the markers
            self.markers_pub_path.publish(marker_path)

def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PoseControl()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()