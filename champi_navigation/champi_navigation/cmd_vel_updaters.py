from math import pi, sqrt, cos, sin

from champi_libraries_py.data_types.geometry import Vel2D
from champi_libraries_py.control.pid import PID
from champi_navigation.path_follow_params import PathFollowParams

from wpimath.trajectory import TrapezoidProfile



# TODO implement robot_angle_when_looking_at_point



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

        # Compute profile constraints
        # TODO update constraints if needed only (for possible better performance)

        self.constraints_mag = TrapezoidProfile.Constraints(p.max_speed_linear, p.max_acc_linear)
        self.constraints_theta = TrapezoidProfile.Constraints(p.max_speed_angular, p.max_acc_angular)

        # Compute angle and magnitude the current heading vector, which express the 2D direction of the robot. The magnitude
        # of the heading vector is the velocity of the robot along the direction of the vector.

        angle_heading_vector = p.robot_state.pose.get_target_angle(p.segment_end)
        robot_vel_along_heading_vector = p.robot_state.vel.x * cos(angle_heading_vector) + p.robot_state.vel.y * sin(angle_heading_vector)

        # Correction of the angle of the heading vector using PID (to follow the line better)

        dist = p.robot_state.pose.dist_to_line_signed(p.segment_start, p.segment_end)

        correction = self.pid_correct_dir.update(dist, 0.1)

        correction_max = 0.1
        if correction > correction_max:
            correction = correction_max
        elif correction < -correction_max:
            correction = -correction_max

        angle_heading_vector += correction

        # Compute magnitude of the velocity

        dist_robot_to_goal = sqrt((p.segment_end.x - p.robot_state.pose.x) ** 2 + (p.segment_end.y - p.robot_state.pose.y) ** 2)

        current_state_mag = TrapezoidProfile.State(-dist_robot_to_goal, robot_vel_along_heading_vector)
        goal_state_mag = TrapezoidProfile.State(0, p.arrival_speed)
        profile_mag = TrapezoidProfile(self.constraints_mag, goal_state_mag, current_state_mag)
        cmd_vel_along_vec = profile_mag.calculate(0.1).velocity

        # Compute theta velocity

        theta_error = 0

        if p.look_at_point is not None:
            theta_error = p.robot_state.pose.get_angle_difference_to_look_at(p.look_at_point)
        else:
            theta_error = p.robot_state.pose.get_angle_difference(p.segment_end)

        current_state_theta = TrapezoidProfile.State(-theta_error, p.robot_state.vel.theta)
        goal_state_theta = TrapezoidProfile.State(0, 0)
        profile_theta = TrapezoidProfile(self.constraints_theta, goal_state_theta, current_state_theta)
        cmd_vel_theta = profile_theta.calculate(0.1).velocity

        # Fill cmd_vel

        cmd_vel = Vel2D()
        cmd_vel.init_from_mag_ang(cmd_vel_along_vec, angle_heading_vector, cmd_vel_theta)

        return cmd_vel
