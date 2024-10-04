from math import pi, atan2, sqrt, cos, sin

from champi_navigation.pid import PID
import champi_navigation.utils as cu

from wpimath.trajectory import TrapezoidProfile


class CmdVelUpdaterInterface:

    """
    Interface for the CmdVelUpdater classes. This interface defines the method compute_cmd_vel that must be implemented.
    This method takes a PathFollowParams object as argument and returns a Vel object that represents the velocity to
    apply to the robot to follow the objectives defined in the PathFollowParams object. The method must be called
    periodically to update the velocity of the robot.
    """

    def compute_cmd_vel(self, p: cu.PathFollowParams):
        pass


class CmdVelUpdaterWPILib(CmdVelUpdaterInterface):
    def __init__(self):

        self.constraints_mag = None
        self.constraints_theta = None

        self.pid_correct_dir = PID(5, 0, 1)

    def compute_cmd_vel(self, p: cu.PathFollowParams):

        # TODO update constraints if needed
        self.constraints_mag = TrapezoidProfile.Constraints(p.max_speed_linear, p.max_acc_linear)
        self.constraints_theta = TrapezoidProfile.Constraints(p.max_speed_angular, p.max_acc_angular)


        angle_vec_dir = atan2(p.segment_end.y - p.robot_state.pose.y, p.segment_end.x - p.robot_state.pose.x)
        dist_robot_to_goal = sqrt((p.segment_end.x - p.robot_state.pose.x) ** 2 + (p.segment_end.y - p.robot_state.pose.y) ** 2)
        # robot vel along the vector between robot and goal
        robot_vel_along_vec = p.robot_state.vel.x * cos(angle_vec_dir) + p.robot_state.vel.y * sin(angle_vec_dir)

        # Compute magnitude of the velocity

        current_state_mag = TrapezoidProfile.State(-dist_robot_to_goal, robot_vel_along_vec)
        goal_state_mag = TrapezoidProfile.State(0, p.arrival_speed)
        profile_mag = TrapezoidProfile(self.constraints_mag, goal_state_mag, current_state_mag)
        cmd_vel_along_vec = profile_mag.calculate(0.1).velocity

        # Compute angular velocity
        theta_error = p.segment_end.theta - p.robot_state.pose.theta
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
        if not p.robot_state.pose.position_equals(p.segment_start): # TODO do we need to test that ?
            dist = p.robot_state.pose.dist_to_line_signed(p.segment_start, p.segment_end)

        correction = self.pid_correct_dir.update(dist, 0.1)

        correction_max = 0.1
        if correction > correction_max:
            correction = correction_max
        elif correction < -correction_max:
            correction = -correction_max

        angle_vec_dir += correction

        # Fill cmd_vel
        cmd_vel = cu.Vel2D()
        cmd_vel.init_from_mag_ang(cmd_vel_along_vec, angle_vec_dir, cmd_vel_theta)

        return cmd_vel


    # def get_theta_error_seg_end(robot_pose, seg_end_pose): TODO
    #     pass

        
        




"""
TODO: Create a new CmdVelUpdater that features "look_at_point". Je pense que c'est la meilleure façon de le faire.
Il faudrait revoir certaines choses: par exemple, tourner avant de commencer à rouler ? sinon le look at point ne 
va pas être respecté au début.

Copiers-collers de l'ancienne implémentation, retirée pour l'instant:


    def calculate_angle_to_target(robot_pose, target_pose: Pose):
        x_robot = robot_pose[0]
        y_robot = robot_pose[1]
        yaw_robot = robot_pose[2]
        x_target = target_pose.position.x
        y_target = target_pose.position.y

        delta_x = x_target - x_robot
        delta_y = y_target - y_robot

        target_angle = atan2(delta_y, delta_x)

        delta_theta = target_angle - yaw_robot

        # normalize angle between [-pi and pi]
        delta_theta = atan2(sin(delta_theta), cos(delta_theta))

        return delta_theta + yaw_robot


        -------------------------------------------------------------------------
        

        if current_champi_segment.do_look_at_point:
            p.arrival_angle = self.calculate_angle_to_target(robot_current_state.pose, current_champi_segment.look_at_point.pose)
        else:
            p.arrival_angle = self.get_arrival_angle()

        
            
        -------------------------------------------------------------------------
        

    def is_current_goal_reached(self, robot_current_state):
        "Checks if the goal is reached and switch to the next one if it is the case.
        Should not be called if i_goal is None = no path to follow."

        current_champi_segment: ChampiSegment = self.champi_path.segments[self.i_goal-1]
        
        error_max_lin = current_champi_segment.end.linear_tolerance
        error_max_ang = current_champi_segment.end.angular_tolerance


        if not current_champi_segment.do_look_at_point:
            target_angle = self.current_seg_end[2]
        else:
            target_angle = self.calculate_angle_to_target(robot_current_state.pose, current_champi_segment.look_at_point.pose)

        angle_ok = self.check_angle(robot_current_state.pose[2], target_angle, error_max_ang)
        
        return (abs(robot_current_state.pose[0] - self.current_seg_end[0]) < error_max_lin
                and abs(robot_current_state.pose[1] - self.current_seg_end[1]) < error_max_lin
                and angle_ok)


"""
