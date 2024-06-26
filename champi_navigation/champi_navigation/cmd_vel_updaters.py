from math import pi, atan2, sqrt, cos, sin

from champi_navigation.pid import PID
from champi_navigation.utils import Vel, dist_point_to_line_signed, PathFollowParams

from wpimath.trajectory import TrapezoidProfile


class CmdVelUpdaterInterface:

    """
    Interface for the CmdVelUpdater classes. This interface defines the method compute_cmd_vel that must be implemented.
    This method takes a PathFollowParams object as argument and returns a Vel object that represents the velocity to
    apply to the robot to follow the objectives defined in the PathFollowParams object. The method must be called
    periodically to update the velocity of the robot.
    """

    def compute_cmd_vel(self, p: PathFollowParams):
        pass


class CmdVelUpdaterWPILib(CmdVelUpdaterInterface):
    def __init__(self):

        self.constraints_mag = None
        self.constraints_theta = None

        self.pid_correct_dir = PID(5, 0, 1)

    def compute_cmd_vel(self, p: PathFollowParams):

        # TODO update constraints if needed
        self.constraints_mag = TrapezoidProfile.Constraints(p.max_speed_linear, p.max_acc_linear)
        self.constraints_theta = TrapezoidProfile.Constraints(p.max_speed_angular, p.max_acc_angular)


        angle_vec_dir = atan2(p.segment_end[1] - p.robot_state.pose[1], p.segment_end[0] - p.robot_state.pose[0])
        dist_robot_to_goal = sqrt((p.segment_end[0] - p.robot_state.pose[0]) ** 2 + (p.segment_end[1] - p.robot_state.pose[1]) ** 2)
        # robot vel along the vector between robot and goal
        robot_vel_along_vec = p.robot_state.vel.x * cos(angle_vec_dir) + p.robot_state.vel.y * sin(angle_vec_dir)

        # Compute magnitude of the velocity

        current_state_mag = TrapezoidProfile.State(-dist_robot_to_goal, robot_vel_along_vec)
        goal_state_mag = TrapezoidProfile.State(0, p.arrival_speed)
        profile_mag = TrapezoidProfile(self.constraints_mag, goal_state_mag, current_state_mag)
        cmd_vel_along_vec = profile_mag.calculate(0.1).velocity

        # Compute angular velocity
        theta_error = p.arrival_angle - p.robot_state.pose[2]
        if abs(theta_error) > pi:
            if theta_error > 0:
                theta_error -= 2 * pi
            else:
                theta_error += 2 * pi

        current_state_theta = TrapezoidProfile.State(-theta_error, p.robot_state.vel.theta)
        goal_state_theta = TrapezoidProfile.State(0, 0)
        profile_theta = TrapezoidProfile(self.constraints_theta, goal_state_theta, current_state_theta)
        cmd_vel_theta = profile_theta.calculate(0.1).velocity

        dist = 0
        if p.robot_state.pose[:2] != p.segment_start[:2]:
            dist = dist_point_to_line_signed(p.robot_state.pose[:2], [p.segment_start[:2], p.segment_end[:2]])

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


