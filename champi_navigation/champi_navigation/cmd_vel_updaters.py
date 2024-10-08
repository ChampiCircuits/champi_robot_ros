from math import pi, sqrt, cos, sin

from champi_libraries_py.data_types.geometry import Vel2D
from champi_libraries_py.control.pid import PID
from champi_navigation.path_follow_params import PathFollowParams

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
        self.pid_correct_dir = PID(5, 0, 1)


    def compute_cmd_vel(self, p: PathFollowParams):


        # ====================== Heading of the robot ======================

        # Here, we compute heading of the robot. This is an angle that represents the direction in which the robot is going.

        # 1) Compute angle and magnitude the heading vector, for the robot to go in the direction of the end of the segment.

        angle_heading = p.robot_state.pose.get_target_angle(p.segment_end)

        # 2) Correction of the angle of the heading using PID (to follow the line better)

        # 2.a) Commpute the error "dist".
        # If the robot is further from the line it's supposed to follow, the error is greater.
        dist = p.robot_state.pose.dist_to_line_signed(p.segment_start, p.segment_end)

        # 2.b) PID correction
        correction = self.pid_correct_dir.update(dist, 0.1)

        # 2.c) Limit the correction.
        correction_max = 0.1
        if correction > correction_max:
            correction = correction_max
        elif correction < -correction_max:
            correction = -correction_max

        # 2.d) We apply the correction to the angle of the heading vector. That means the robot
        # will move slightly more to the left or to the right, to get closer to the line.
        angle_heading += correction


        # ======================= XY Linear velocity =========================

        # 1) Compute distance error to the goal (end of the segment)
        dist_robot_to_goal = sqrt((p.segment_end.x - p.robot_state.pose.x) ** 2 + (p.segment_end.y - p.robot_state.pose.y) ** 2)

        # 2) Compute CURRENT robot speed along the heading vector (2D vector oriented by angle_heading)
        x_y_speed = p.robot_state.vel.x * cos(angle_heading) + p.robot_state.vel.y * sin(angle_heading)

        # 3) We use a trapezoidal profile that tracks the distance error to the goal.
        # We give distance error as start, 0 as goal and current speed, and we get the speed for next time step.
        constraints_mag = TrapezoidProfile.Constraints(p.max_speed_linear, p.max_acc_linear)
        current_state_mag = TrapezoidProfile.State(-dist_robot_to_goal, x_y_speed)
        goal_state_mag = TrapezoidProfile.State(0, p.end_speed)
        profile_mag = TrapezoidProfile(constraints_mag, goal_state_mag, current_state_mag)
        cmd_vel_x_y = profile_mag.calculate(0.1).velocity


        # ========================= Angular velocity =========================

        # 1) Compute angular error between the robot and the targeted angle.
        # All the following angles are expressed relative to the global frame.

        # In the normal case, the targeted angle is the angle from the end pose.

        # In the case of look_at_point enabled, the targeted angle is the angle of the segment [robot, look_at_point].
        # We also substract to this error the angle "robot_angle_when_looking_at_point", for the robot to face the
        # look_at_point with the desired angle.


        theta_error = 0

        if p.look_at_point is not None:  # Look at point mode
            theta_error = p.robot_state.pose.get_angle_difference_to_look_at(p.look_at_point, p.robot_angle_when_looking_at_point)
        else:  # Normal mode
            theta_error = p.robot_state.pose.get_angle_difference(p.segment_end)

        # 2) We use a trapezoidal profile that tracks the angle error to the targeted angle.
        # We give current angle error as start, 0 as goal and current speed, and we get the angular speed for next time step.
        constraints_theta = TrapezoidProfile.Constraints(p.max_speed_angular, p.max_acc_angular)
        current_state_theta = TrapezoidProfile.State(-theta_error, p.robot_state.vel.theta)
        goal_state_theta = TrapezoidProfile.State(0, 0)
        profile_theta = TrapezoidProfile(constraints_theta, goal_state_theta, current_state_theta)
        cmd_vel_theta = profile_theta.calculate(0.1).velocity  # TODO pass dt as argument

        # ========================= Final velocity command =========================

        # All left to do is compute the the velocity command in the ros format (linear x, linear y, angular z).
        # The heading angle and the x_y_speed can be thought as the direction and the magnitude of a 2D velocity vector.

        cmd_vel = Vel2D()
        cmd_vel.init_from_mag_ang(cmd_vel_x_y, angle_heading, cmd_vel_theta)

        return cmd_vel
