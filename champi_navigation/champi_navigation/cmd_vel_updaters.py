from math import pi, atan2, sqrt, cos, sin
import time

from champi_navigation.pid import PID
from champi_navigation.trapezoidal_velocity_profile import TrapezoidalVelocityProfile
from champi_navigation.utils import Vel, dist_point_to_line_signed

from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians, PIDController
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians

from icecream import ic


class CmdVelUpdaterWPILib:
    def __init__(self):

        self.constraints_mag = TrapezoidProfile.Constraints(0.5, 0.5)
        self.constraints_theta = TrapezoidProfile.Constraints(3., 1.)

        self.pid_correct_dir = PID(5, 0, 1)


    def compute_cmd_vel(self, robot_state, pose_prev, pose_goal, arrival_speed, arrival_angle):

        angle_vec_dir = atan2(pose_goal[1] - robot_state.pose[1], pose_goal[0] - robot_state.pose[0])
        dist_robot_to_goal = sqrt((pose_goal[0] - robot_state.pose[0]) ** 2 + (pose_goal[1] - robot_state.pose[1]) ** 2)
        # robot vel along the vector between robot and goal
        robot_vel_along_vec = robot_state.vel.x * cos(angle_vec_dir) + robot_state.vel.y * sin(angle_vec_dir)

        # Compute magnitude of the velocity

        current_state_mag = TrapezoidProfile.State(-dist_robot_to_goal, robot_vel_along_vec)
        goal_state_mag = TrapezoidProfile.State(0, arrival_speed)
        profile_mag = TrapezoidProfile(self.constraints_mag, goal_state_mag, current_state_mag)
        cmd_vel_along_vec = profile_mag.calculate(0.1).velocity

        # Compute angular velocity
        theta_error = arrival_angle - robot_state.pose[2]
        if abs(theta_error) > pi:
            if theta_error > 0:
                theta_error -= 2 * pi
            else:
                theta_error += 2 * pi

        current_state_theta = TrapezoidProfile.State(-theta_error, robot_state.vel.theta)
        goal_state_theta = TrapezoidProfile.State(0, 0)
        profile_theta = TrapezoidProfile(self.constraints_theta, goal_state_theta, current_state_theta)
        cmd_vel_theta = profile_theta.calculate(0.1).velocity



        dist = 0
        if robot_state.pose[:2] != pose_prev[:2]:
            dist = dist_point_to_line_signed(robot_state.pose[:2], [pose_prev[:2], pose_goal[:2]])

        correction = self.pid_correct_dir.update(dist, 0.1)

        correction_max = 0.1
        if correction > correction_max:
            correction = correction_max
        elif correction < -correction_max:
            correction = -correction_max

        angle_vec_dir += correction



        # Fill cmd_vel
        cmd_vel = Vel()
        cmd_vel.init_from_mag_ang(cmd_vel_along_vec, angle_vec_dir, cmd_vel_theta)

        return cmd_vel


