from math import pi, atan2, sqrt
import time

from champi_navigation.pid import PID
from champi_navigation.trapezoidal_velocity_profile import TrapezoidalVelocityProfile
from champi_navigation.utils import Vel, dist_point_to_line_signed



class CmdVelUpdaterPID:
    def __init__(self):
        # PIDs
        self.pid_pos_x = PID(1, 0, 0)
        self.pid_pos_y = PID(1, 0, 0)
        self.pid_pos_theta = PID(1, 0, 0,)

        self.last_time_called = time.time()


    def compute_cmd_vel(self, robot_pose, pose_goal):
        dt = time.time() - self.last_time_called
        self.last_time_called = time.time()

        # if it's shorter to turn in the other direction, we do it
        # TODO not always working
        error_theta = pose_goal[2] - robot_pose[2]
        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta -= 2*pi
            else:
                error_theta += 2*pi

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

            distance_start_to_goal = ((pose_goal[0] - self.start_pose[0])**2 + (pose_goal[1] - self.start_pose[1])**2)**0.5
            # distance_start_to_goal = ((pose_goal[0] - self.start_pose.pose.position.x)**2 + (pose_goal[1] - self.start_pose.pose.position.y)**2)**0.5
            current_mag_speed = 0
            if self.vel_profile_mag.t_start is not None:
                current_mag_speed = self.vel_profile_mag.compute_vel(None)
            self.vel_profile_mag.set_new_goal(distance_start_to_goal, 0, current_mag_speed) # TODO pareil pour theta?

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
        theta=0 # TODO REMETTRE

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