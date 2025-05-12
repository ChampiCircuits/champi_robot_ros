#!/usr/bin/env python3

from math import sqrt, cos, sin, atan2
import time
import numpy

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from tf2_ros import TransformBroadcaster


from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped


from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped

from champi_libraries_py.utils.angles import get_yaw


class HoloBaseControlDummy(Node):

    def __init__(self):
        super().__init__('holo_base_control_dummy_node')

        # Parameters
        self.declare_parameter('enable_accel_limits', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('max_acceleration_linear', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_acceleration_angular', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_deceleration_linear', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_deceleration_angular', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_acceleration_wheel', rclpy.Parameter.Type.DOUBLE)
        # Get parameters
        self.enable_accel_limits_ = self.get_parameter('enable_accel_limits').value
        self.max_acceleration_linear_ = self.get_parameter('max_acceleration_linear').value
        self.max_acceleration_angular_ = self.get_parameter('max_acceleration_angular').value
        self.max_deceleration_linear_ = self.get_parameter('max_deceleration_linear').value
        self.max_deceleration_angular_ = self.get_parameter('max_deceleration_angular').value
        self.max_acceleration_wheel = self.get_parameter('max_acceleration_wheel').value
        # Print parameters
        get_logger('rclpy').info(f"enable_accel_limits: {self.enable_accel_limits_}")
        get_logger('rclpy').info(f"max_acceleration_linear: {self.max_acceleration_linear_}")
        get_logger('rclpy').info(f"max_acceleration_angular: {self.max_acceleration_angular_}")
        get_logger('rclpy').info(f"max_deceleration_linear: {self.max_deceleration_linear_}")
        get_logger('rclpy').info(f"max_deceleration_angular: {self.max_deceleration_angular_}")
        get_logger('rclpy').info(f"max_acceleration_wheel: {self.max_acceleration_wheel}")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.pub = self.create_publisher(Odometry, '/odom_otos', 10)

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Node variables
        self.latest_cmd_vel = [0., 0., 0.]
        self.current_vel = [0., 0., 0.]

        self.current_pose = [1.5, 0.5, 0.]

        self.first_time = True

        self.speed_wheel0 = 0
        self.speed_wheel1 = 0
        self.speed_wheel2 = 0

        self.robot_radius = 0.175

        self.dt_ = 0.02
        self.last_time_ = 0.0

        self.t_last_cmd_vel_ = 0.0

      
    def listener_callback(self, msg):
        self.latest_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.t_last_cmd_vel_ = time.time()

        # add small noise to the cmd_vel to resemble the real world
        self.latest_cmd_vel[0] += numpy.random.normal(0, 0.002)
        self.latest_cmd_vel[1] += numpy.random.normal(0, 0.002)
        self.latest_cmd_vel[2] += numpy.random.normal(0, 0.01)

    # Returns the velocity with limited acceleration applied
    def limit_accel(self, current_speed, goal_speed, max_acceleration, dt):
        # check if goal is smaller or greater than current speed
        if goal_speed > current_speed:
            # Compute the speed we can reach in dt
            max_speed = current_speed + max_acceleration * dt
            # Check if we can reach the goal speed
            if max_speed < goal_speed:
                # We can't reach the goal speed
                return max_speed
            else:
                # We can reach the goal speed
                return goal_speed
        else:
            # Compute the speed we can reach in dt
            min_speed = current_speed - max_acceleration * dt
            # Check if we can reach the goal speed
            if min_speed > goal_speed:
                # We can't reach the goal speed
                return min_speed
            else:
                # We can reach the goal speed
                return goal_speed

    # Returns the velocity with limited deceleration applied
    def limit_decel(self, current_speed, goal_speed, max_deceleration, dt):
        # check if goal is smaller or greater than current speed
        if goal_speed > current_speed:
            # Compute the speed we can reach in dt
            max_speed = current_speed + max_deceleration * dt
            # Check if we can reach the goal speed
            if max_speed < goal_speed:
                # We can't reach the goal speed
                return max_speed
            else:
                # We can reach the goal speed
                return goal_speed
        else:
            # Compute the speed we can reach in dt
            min_speed = current_speed - max_deceleration * dt
            # Check if we can reach the goal speed
            if min_speed > goal_speed:
                # We can't reach the goal speed
                return min_speed
            else:
                # We can reach the goal speed
                return goal_speed

    # Returns the velocity with limited acceleration and deceleration applied
    def limit_accel_decel(self, current_speed, goal_speed, max_acceleration, max_deceleration, dt):
        # Check if we are at constant speed, accelerating or decelerating
        if goal_speed == current_speed:
            # We are at constant speed
            return current_speed
        elif (goal_speed > current_speed and current_speed >= 0) or (goal_speed < current_speed and current_speed <= 0):
            # We are accelerating
            return self.limit_accel(current_speed, goal_speed, max_acceleration, dt)
        else:
            # We are decelerating
            return self.limit_decel(current_speed, goal_speed, max_deceleration, dt)


            
    def timer_callback(self):

        # if no cmd_vel received for 0.5s, set cmd_vel to 0
        if time.time() - self.t_last_cmd_vel_ > 0.5:
            self.latest_cmd_vel = [0., 0., 0.]

        # Compute dt
        if self.last_time_ == 0.0:
            self.last_time_ = time.time()
            return
        dt = time.time() - self.last_time_
        self.last_time_ = time.time()

        # Limit acceleration
        if self.enable_accel_limits_:
            # Limit linear acceleration xy
            current_speed = sqrt(self.current_vel[0]**2 + self.current_vel[1]**2)
            goal_speed = sqrt(self.latest_cmd_vel[0]**2 + self.latest_cmd_vel[1]**2)
            cmd_vxy_limited = self.limit_accel_decel(current_speed, goal_speed, self.max_acceleration_linear_, self.max_deceleration_linear_, dt)

            if goal_speed == 0:
                # Use angle of the current vel of the robot. This is to avoid the robot to go forward when the goal speed is 0 but the robot is still moving
                vel_vect_angle = atan2(self.current_vel[1], self.current_vel[0])
            else:
                vel_vect_angle = atan2(self.latest_cmd_vel[1], self.latest_cmd_vel[0])
            cmd_vx_limited = cmd_vxy_limited * cos(vel_vect_angle)
            cmd_vy_limited = cmd_vxy_limited * sin(vel_vect_angle)

            # Limit angular acceleration z
            cmd_wz_limited = self.limit_accel_decel(self.current_vel[2], self.latest_cmd_vel[2], self.max_acceleration_angular_, self.max_deceleration_angular_, dt)
        else:
            cmd_vx_limited = self.latest_cmd_vel[0]
            cmd_vy_limited = self.latest_cmd_vel[1]
            cmd_wz_limited = self.latest_cmd_vel[2]

        self.cmd_vel_to_wheels(cmd_vx_limited, cmd_vy_limited, cmd_wz_limited)
        self.wheels_to_current_vel()

        self.update_pose()

        # Publish the odometry # TODO rotation and rotation vel are not taken in account by ukf
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.current_vel[0]
        odom.twist.twist.linear.y = self.current_vel[1]
        odom.twist.twist.angular.z = self.current_vel[2]
        odom.pose.pose.position.x = self.current_pose[0]
        odom.pose.pose.position.y = self.current_pose[1]
        odom.pose.pose.position.z = 0.
        odom.pose.pose.orientation.x = 0.
        odom.pose.pose.orientation.y = 0.
        odom.pose.pose.orientation.z = sin(self.current_pose[2] / 2)
        odom.pose.pose.orientation.w = cos(self.current_pose[2] / 2)
        self.pub.publish(odom)



    def wheels_to_current_vel(self):
        # Convert from WHEEL speeds --> LINEAR and ANGULAR speeds
        self.current_vel[0] = -(self.speed_wheel0 + self.speed_wheel1 - 2*self.speed_wheel2)
        self.current_vel[1] = 1/3*(-self.speed_wheel0*sqrt(3) + self.speed_wheel1*sqrt(3))
        self.current_vel[2] = (1 / (self.robot_radius)) * \
            (-self.speed_wheel0 - self.speed_wheel1 + self.speed_wheel2)

    def cmd_vel_to_wheels(self, cmd_vx, cmd_vy, cmd_wz):
        # Convert from LINEAR and ANGULAR speeds --> WHEEL speeds
        cmd_vitesse_roue0 = 0.5 * cmd_vx - sqrt(3) / 2 * cmd_vy - self.robot_radius * cmd_wz
        cmd_vitesse_roue1 = 0.5 * cmd_vx + sqrt(3) / 2 * cmd_vy - self.robot_radius * cmd_wz
        cmd_vitesse_roue2 = cmd_vx - self.robot_radius * cmd_wz


        # limit the speeds
        accel_roue_0 = cmd_vitesse_roue0 - self.speed_wheel0
        accel_roue_1 = cmd_vitesse_roue1 - self.speed_wheel1
        accel_roue_2 = cmd_vitesse_roue2 - self.speed_wheel2
        
        abs_accel_roue_0 = abs(accel_roue_0)
        abs_accel_roue_1 = abs(accel_roue_1)
        abs_accel_roue_2 = abs(accel_roue_2)
        abs_accel_roues = [abs_accel_roue_0, abs_accel_roue_1, abs_accel_roue_2]

        max_accel_per_cycle = self.max_acceleration_wheel * self.timer_period

        if abs_accel_roue_0 < max_accel_per_cycle and abs_accel_roue_1 < max_accel_per_cycle and abs_accel_roue_2 < max_accel_per_cycle:
            # acceleration requested is ok, no need to accelerate gradually.
            self.speed_wheel0 = cmd_vitesse_roue0
            self.speed_wheel1 = cmd_vitesse_roue1
            self.speed_wheel2 = cmd_vitesse_roue2
        else:
            speed_ratio = max_accel_per_cycle / max(abs_accel_roues)
            self.speed_wheel0 += speed_ratio * accel_roue_0
            self.speed_wheel1 += speed_ratio * accel_roue_1
            self.speed_wheel2 += speed_ratio * accel_roue_2


    def update_pose(self):
        # Update current_pose_ by integrating current_vel_

        if self.first_time:
            self.last_time = time.time()
            self.first_time = False
            return

        dt = time.time() - self.last_time
        self.last_time = time.time()

        # Robot speed is expressed relative to the robot frame, so we need to transform it to the world frame
        self.current_pose[0] += self.current_vel[0] * cos(self.current_pose[2]) * dt - self.current_vel[1] * sin(self.current_pose[2]) * dt
        self.current_pose[1] += self.current_vel[0] * sin(self.current_pose[2]) * dt + self.current_vel[1] * cos(self.current_pose[2]) * dt
        self.current_pose[2] += self.current_vel[2] * dt


def main(args=None):
    rclpy.init(args=args)

    node = HoloBaseControlDummy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()