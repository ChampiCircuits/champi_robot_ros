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
        self.constraints_theta = TrapezoidProfile.Constraints(3., 3.)

        self.pid_correct_dir = PID(2, 0, 1)


    def compute_cmd_vel(self, robot_state, pose_goal, arrival_speed):

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
        theta_error = pose_goal[2] - robot_state.pose[2]
        if abs(theta_error) > pi:
            if theta_error > 0:
                theta_error -= 2 * pi
            else:
                theta_error += 2 * pi

        current_state_theta = TrapezoidProfile.State(-theta_error, robot_state.vel.theta)
        goal_state_theta = TrapezoidProfile.State(0, 0)
        profile_theta = TrapezoidProfile(self.constraints_theta, goal_state_theta, current_state_theta)
        cmd_vel_theta = profile_theta.calculate(0.1).velocity

        # Fill cmd_vel
        cmd_vel = Vel()
        cmd_vel.init_from_mag_ang(cmd_vel_along_vec, angle_vec_dir, cmd_vel_theta)





        return cmd_vel


class CmdVelUpdaterWPILib3:
    def __init__(self):

        self.constraints_theta = TrapezoidProfileRadians.Constraints(3., 3.)
        self.constraints_x = TrapezoidProfile.Constraints(0.5, 0.5)
        self.constraints_y = TrapezoidProfile.Constraints(0.5, 0.5)

        self.pid_vel_x = PIDController(2., 0, 0)
        self.pid_vel_y = PIDController(2., 0, 0)

    def compute_cmd_vel(self, robot_state, pose_goal):

        cmd_vel = Vel()

        current_state_x = TrapezoidProfile.State(robot_state.pose[0], robot_state.vel.x)
        current_state_y = TrapezoidProfile.State(robot_state.pose[1], robot_state.vel.y)
        current_state_theta = TrapezoidProfileRadians.State(robot_state.pose[2], robot_state.vel.theta)

        goal_state_x = TrapezoidProfile.State(pose_goal[0], 0)
        goal_state_y = TrapezoidProfile.State(pose_goal[1], 0)
        goal_state_theta = TrapezoidProfileRadians.State(pose_goal[2], 0)

        profile_x = TrapezoidProfile(self.constraints_x, goal_state_x, current_state_x)
        profile_y = TrapezoidProfile(self.constraints_y, goal_state_y, current_state_y)
        profile_theta = TrapezoidProfileRadians(self.constraints_theta, goal_state_theta, current_state_theta)


        cmd_vel.x = profile_x.calculate(0.1).velocity
        cmd_vel.y = profile_y.calculate(0.1).velocity
        cmd_vel.theta = profile_theta.calculate(0.1).velocity

        # cmd_vel.x = 0
        # cmd_vel.y = 0
        cmd_vel.theta = 0


        return cmd_vel

class CmdVelUpdaterWPILib2:
    def __init__(self):

        self.controller_pos_x = ProfiledPIDController(
            10, 0, 0,
            TrapezoidProfile.Constraints(0.5, 1.))

        self.controller_pos_y = ProfiledPIDController(
            10, 0, 0,
            TrapezoidProfile.Constraints(0.5, 1.0))

        self.controller_pos_theta = ProfiledPIDControllerRadians(
            1, 0, 0,
            TrapezoidProfileRadians.Constraints(2., 3.))


    def compute_cmd_vel(self, robot_state, pose_goal):

        cmd_vel = Vel()

        # current_state_x = TrapezoidProfile.State(robot_state.pose[0], robot_state.vel.x)
        # current_state_y = TrapezoidProfile.State(robot_state.pose[1], robot_state.vel.y)
        # current_state_theta = TrapezoidProfileRadians.State(robot_state.pose[2], robot_state.vel.theta)
        #
        # goal_state_x = TrapezoidProfile.State(pose_goal[0], 0)
        # goal_state_y = TrapezoidProfile.State(pose_goal[1], 0)
        # goal_state_theta = TrapezoidProfileRadians.State(pose_goal[2], 0)


        cmd_vel.x = self.controller_pos_x.calculate(robot_state.pose[0], pose_goal[0])
        cmd_vel.y = self.controller_pos_y.calculate(robot_state.pose[1], pose_goal[1])
        cmd_vel.theta = self.controller_pos_theta.calculate(robot_state.pose[2], pose_goal[2])

        return cmd_vel



class CmdVelUpdaterPID:
    def __init__(self):
        # PIDs
        self.pid_pos_x = PID(1, 0, 0)
        self.pid_pos_y = PID(1, 0, 0)
        self.pid_pos_theta = PID(1, 0, 0, )

        self.last_time_called = time.time()

    def compute_cmd_vel(self, robot_pose, pose_goal):
        dt = time.time() - self.last_time_called
        self.last_time_called = time.time()

        # if it's shorter to turn in the other direction, we do it
        # TODO not always working
        error_theta = pose_goal[2] - robot_pose[2]
        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta -= 2 * pi
            else:
                error_theta += 2 * pi

        cmd_vel = Vel()

        cmd_vel.theta = self.pid_pos_theta.update(error_theta, dt)

        # # PID
        cmd_vel.x = self.pid_pos_x.update(pose_goal[0] - robot_pose[0], dt)
        cmd_vel.y = self.pid_pos_y.update(pose_goal[1] - robot_pose[1], dt)

        return cmd_vel


class CmdVelUpdaterTrapVelProfile:
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
            # print("start_pose",self.start_pose)
            # print("goal_pose",self.goal_pose)

            distance_start_to_goal = ((pose_goal[0] - self.start_pose[0]) ** 2 + (
                        pose_goal[1] - self.start_pose[1]) ** 2) ** 0.5
            # distance_start_to_goal = ((pose_goal[0] - self.start_pose.pose.position.x)**2 + (pose_goal[1] - self.start_pose.pose.position.y)**2)**0.5
            current_mag_speed = 0
            if self.vel_profile_mag.t_start is not None:
                current_mag_speed = self.vel_profile_mag.compute_vel(None)
            self.vel_profile_mag.set_new_goal(distance_start_to_goal, 0, current_mag_speed)  # TODO pareil pour theta?

            # if it's shorter to turn in the other direction, we do it
            error_theta = pose_goal[2] - robot_current_pose[2]
            if abs(error_theta) > pi:
                if error_theta > 0:
                    error_theta -= 2 * pi
                else:
                    error_theta += 2 * pi

            self.vel_profile_theta.set_new_goal(error_theta, 0)

        # Compute distance to goal
        # distance_to_goal = ((pose_goal[0] - robot_current_pose[0])**2 + (pose_goal[1] - robot_current_pose[1])**2)**0.5

        mag = self.vel_profile_mag.compute_vel(None)
        theta = self.vel_profile_theta.compute_vel(None)
        theta = 0  # TODO REMETTRE

        angle_vec_dir = atan2(pose_goal[1] - robot_current_pose[1], pose_goal[0] - robot_current_pose[0])

        dist = 0
        if robot_current_pose[:2] != self.start_pose[:2]:  # sinon div par 0
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
