from math import pi, sqrt, cos, sin, trunc, copysign, atan2

from champi_libraries_py.data_types.geometry import Vel2D
from champi_libraries_py.control.pid import PID
from champi_navigation.path_follow_params import PathFollowParams

from wpimath.trajectory import TrapezoidProfile

import diagnostic_msgs
from icecream import ic

class CmdVelUpdaterInterface:
    """
    Abstract class for the CmdVelUpdater classes. This interface defines the method compute_cmd_vel that must be implemented.
    This method takes a PathFollowParams object as argument and returns a Vel object that represents the velocity to
    apply to the robot to follow the objectives defined in the PathFollowParams object. The method must be called
    periodically to update the velocity of the robot.
    """

    def __init__(self):
        # Variables to store statistics, for diagnostics
        self.stat_error_dist = None
        self.stat_error_theta = None


    def compute_cmd_vel(self, dt, p: PathFollowParams):
        """See class description.

        Args:
            p (PathFollowParams): See class description.
        """
        pass


    def produce_diagnostics(self, stat):
        """Callback method to be used in with a DiagnosticUpdater object.

        Args:
            stat (DiagnosticStatus): No need to worry about that, the diagnostic updater takes care of it.

        Returns:
            DiagnosticStatus: No need to worry about that, the diagnostic updater takes care of it.
        """
        if self.stat_error_dist is not None and self.stat_error_theta is not None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'CmdVelUpdater is running')
            stat.add('Error dist (m)', str(self.stat_error_dist))
            stat.add('Error theta (rad)', str(self.stat_error_theta))
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Statistics not available')
            stat.add('Error dist (m)', 'N/A')
            stat.add('Error theta (rad)', 'N/A')
        return stat


class CmdVelUpdaterWPILib(CmdVelUpdaterInterface):
    def __init__(self):
        super().__init__()
        self.pid_correct_dir = PID(5, 0, 1)


    def compute_cmd_vel(self, dt, p: PathFollowParams):

        # ======================= XY Linear velocity =========================

        # 1) Compute distance error to the goal (end of the segment)
        dist_robot_to_goal_x = p.segment_end.x - p.robot_state.pose.x
        dist_robot_to_goal_y = p.segment_end.y - p.robot_state.pose.y

        # 2.a) Compute CURRENT robot speed along global x and y axes.
        robot_speed_global = p.robot_state.vel #Vel2D.to_global_frame(p.robot_state.pose, p.robot_state.vel)

        # 2.b) Compute end speed along each axis (projection of end speed on the segment).
        angle_segment = atan2(dist_robot_to_goal_y, dist_robot_to_goal_x)
        end_speed_x = p.end_speed * cos(angle_segment)
        end_speed_y = p.end_speed * sin(angle_segment)

        # 2.c) Compute max vel along each axis.
        max_speed_x = abs(p.max_speed_linear * cos(angle_segment))
        max_speed_y = abs(p.max_speed_linear * sin(angle_segment))

        # 2.d) Compute max acceleration along each axis.
        max_acc_x = abs(p.max_acc_linear * cos(angle_segment))
        max_acc_y = abs(p.max_acc_linear * sin(angle_segment))

        # 3) We use 2 trapezoidal profiles, one for x and one for y.

        constraints_x = TrapezoidProfile.Constraints(max_speed_x, max_acc_x)
        current_state_x = TrapezoidProfile.State(p.robot_state.pose.x, robot_speed_global.x)
        goal_state_x = TrapezoidProfile.State(p.segment_end.x, end_speed_x)
        profile_x = TrapezoidProfile(constraints_x)
        cmd_vel_x = profile_x.calculate(dt, current_state_x, goal_state_x).velocity

        constraints_y = TrapezoidProfile.Constraints(max_acc_y, max_speed_y)
        current_state_y = TrapezoidProfile.State(p.robot_state.pose.y, robot_speed_global.y)
        goal_state_y = TrapezoidProfile.State(p.segment_end.y, end_speed_y)
        profile_y = TrapezoidProfile(constraints_y)
        cmd_vel_y = profile_y.calculate(dt, current_state_y, goal_state_y).velocity




        # ========================= Angular velocity =========================

        # 1) Compute angular error between the robot and the targeted angle.
        # All the following angles are expressed relative to the global frame.

        # In the normal case, the targeted angle is the angle from the end pose.

        # In the case of look_at_point enabled, the targeted angle is the angle of the segment [robot, look_at_point].
        # We also substract to this error the angle "robot_angle_when_looking_at_point", for the robot to face the
        # look_at_point with the desired angle.

        if p.look_at_point is not None:  # Look at point mode
            theta_error = p.robot_state.pose.get_angle_difference_to_look_at(p.look_at_point, p.robot_angle_when_looking_at_point)
        else:  # Normal mode
            theta_error = p.robot_state.pose.get_angle_difference(p.segment_end)

        # 2) We use a trapezoidal profile that tracks the angle error to the targeted angle.
        # We give current angle error as start, 0 as goal and current speed, and we get the angular speed for next time step.
        constraints_theta = TrapezoidProfile.Constraints(p.max_speed_angular, p.max_acc_angular)
        current_state_theta = TrapezoidProfile.State(-theta_error, p.robot_state.vel.theta)
        goal_state_theta = TrapezoidProfile.State(0, 0)
        profile_theta = TrapezoidProfile(constraints_theta)
        cmd_vel_theta = profile_theta.calculate(dt, current_state_theta, goal_state_theta).velocity




        # ====================== Correction ======================

        # Compute a vector that is perpendicular to the segment [segment_start, segment_end].
        # The vector start is the robot.
        # We call the vector 'correction_vector'.

        # segment direction
        dx = p.segment_end.x - p.segment_start.x
        dy = p.segment_end.y - p.segment_start.y

        # normalize segment direction
        length = sqrt(dx**2 + dy**2)
        if length == 0:
            perp_x, perp_y = 0.0, 0.0  # Degenerate case: segment is a point
        else:
            # Perpendicular vector (to the left of the segment direction)
            perp_x = -dy / length
            perp_y = dx / length


        # 2.a) Compute the error "dist".
        # If the robot is further from the line it's supposed to follow, the error is greater.
        # But if the line is a point (i.e only a rotation is required), the error is 0.
        if p.segment_start.x == p.segment_end.x and p.segment_start.y == p.segment_end.y:
            dist = 0
        else:
            dist = p.robot_state.pose.dist_to_line_signed(p.segment_start, p.segment_end)


        # One PID per direction (x and y)

        correction_scale = self.pid_correct_dir.update(dist, dt)

        # Limit the correction to a maximum value.
        correction_max = 0.1
        if correction_scale > correction_max:
            correction_scale = correction_max
            ic('Correction scale limited to max:', correction_max)
        elif correction_scale < -correction_max:
            correction_scale = -correction_max
            ic('Correction scale limited to min:', -correction_max)


        # Apply the correction to the cmd_vel_x and cmd_vel_y.
        cmd_vel_x += correction_scale * perp_x
        cmd_vel_y += correction_scale * perp_y





        # ========================= Final velocity command =========================

        # All left to do is compute the velocity command in the ros format (linear x, linear y, angular z).
        # The heading angle and the x_y_speed can be thought as the direction and the magnitude of a 2D velocity vector.

        cmd_vel = Vel2D()
        cmd_vel.x = cmd_vel_x
        cmd_vel.y = cmd_vel_y
        cmd_vel.theta = cmd_vel_theta


        # ========================= Statistics =========================
        self.stat_error_dist = -1
        self.stat_error_theta = theta_error

        return cmd_vel

class CmdVelUpdaterWPILibMagnitude(CmdVelUpdaterInterface):
    def __init__(self):
        super().__init__()
        self.pid_correct_dir = PID(5, 0, 1)


    def compute_cmd_vel(self, dt, p: PathFollowParams):


        # ====================== Heading of the robot ======================

        # Here, we compute heading of the robot. This is an angle that represents the direction in which the robot is going.

        # 1) Compute angle and magnitude the heading vector, for the robot to go in the direction of the end of the segment.

        angle_heading = p.robot_state.pose.get_target_angle(p.segment_end)

        # 2) Correction of the angle of the heading using PID (to follow the line better)

        # 2.a) Compute the error "dist".
        # If the robot is further from the line it's supposed to follow, the error is greater.
        # But if the line is a point (i.e only a rotation is required), the error is 0
        if p.segment_start.x == p.segment_end.x and p.segment_start.y == p.segment_end.y:
            dist = 0
        else:
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
        profile_mag = TrapezoidProfile(constraints_mag)
        cmd_vel_x_y = profile_mag.calculate(dt, current_state_mag, goal_state_mag).velocity

        # cmd_vel_x_y = compute_cmd_vel_x_y_custom(p, angle_heading, dt)



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
        profile_theta = TrapezoidProfile(constraints_theta)
        cmd_vel_theta = profile_theta.calculate(dt, current_state_theta, goal_state_theta).velocity
        #
        # cmd_vel_theta = compute_cmd_vel_theta_custom(p, theta_error, dt)


        # ========================= Final velocity command =========================

        # All left to do is compute the velocity command in the ros format (linear x, linear y, angular z).
        # The heading angle and the x_y_speed can be thought as the direction and the magnitude of a 2D velocity vector.

        cmd_vel = Vel2D()
        cmd_vel.init_from_mag_ang(cmd_vel_x_y, angle_heading, cmd_vel_theta)


        # ========================= Statistics =========================
        self.stat_error_dist = dist_robot_to_goal
        self.stat_error_theta = theta_error

        return cmd_vel



def compute_cmd_vel_x_y_custom(p, angle_heading, dt):
    dx = p.segment_end.x - p.robot_state.pose.x
    dy = p.segment_end.y - p.robot_state.pose.y
    dist_robot_to_goal = sqrt(dx**2 + dy**2)

    angle_to_target = atan2(dy, dx)
    angle_diff = normalize_angle(angle_to_target - angle_heading)

    vel_init = (
            p.robot_state.vel.x * cos(angle_heading) +
            p.robot_state.vel.y * sin(angle_heading)
    )

    pos_init = dist_robot_to_goal

    v_max = p.max_speed_linear
    a_max = p.max_acc_linear
    d_max = p.max_dec_linear

    # Distance nécessaire pour arrêter
    stopping_distance = (vel_init**2) / (2 * d_max) if d_max != 0 else 0

    if stopping_distance >= pos_init:
        # Freinage
        vel_next = vel_init - copysign(d_max * dt, vel_init)
        if vel_init * vel_next < 0:
            vel_next = 0.0
    else:
        # Accélération
        if abs(vel_init) < v_max:
            vel_next = vel_init + copysign(a_max * dt, v_max - vel_init)
            vel_next = max(min(vel_next, v_max), -v_max)
        else:
            vel_next = vel_init

    # Limiter vitesse en fonction de la distance restante pour éviter overshoot
    max_vel_based_on_error = min(v_max, abs(pos_init) * 3)  # coefficient à régler (plus c'est bas plus c'est amorti)
    vel_next = max(min(vel_next, max_vel_based_on_error), -max_vel_based_on_error)

    # Appliquer le signe selon l’angle relatif
    vel_next = copysign(vel_next, cos(angle_diff))

    return vel_next





def compute_cmd_vel_theta_custom(p, theta_error, dt):
    vel_init = p.robot_state.vel.theta

    v_max = p.max_speed_angular
    a_max = p.max_acc_angular
    d_max = p.max_dec_angular

    pos_init = theta_error
    pos_goal = 0.0

    # Distance nécessaire pour s’arrêter (en angle)
    stopping_distance = (vel_init**2) / (2 * d_max) if d_max != 0 else 0

    # Freinage anticipé si nécessaire
    if stopping_distance >= abs(pos_init):
        # On freine dans le sens opposé à la vitesse actuelle
        vel_next = vel_init - copysign(d_max * dt, vel_init)
        # Ne pas inverser le sens de rotation (éviter oscillations)
        if vel_init * vel_next < 0:
            vel_next = 0.0
    else:
        # On accélère vers la cible dans le bon sens (sens donné par theta_error)
        direction = copysign(1.0, pos_init)
        if abs(vel_init) < v_max:
            vel_next = vel_init + direction * a_max * dt
            # Clamp dans les bornes max/min
            vel_next = max(min(vel_next, v_max), -v_max)
        else:
            vel_next = vel_init

    # Limiter la vitesse en fonction de la distance angulaire restante pour éviter overshoot
    max_vel_based_on_error = min(v_max, abs(pos_init) * 3)  # coefficient à ajuster
    vel_next = max(min(vel_next, max_vel_based_on_error), -max_vel_based_on_error)

    return vel_next


def normalize_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle