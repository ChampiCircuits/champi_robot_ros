#!/usr/bin/env python3


from champi_interfaces.msg import CtrlGoal
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.data_types.robot_state import RobotState


class PathFollowParams:
    """This class stores the parameters used by a CmdVelUpdater class.
    We need to pass this class each time we call compute_cmd_vel.
    """

    def __init__(self, max_acc_linear:float, max_acc_angular:float):
        """Constructor. We pass the maximum accelerations of the robot, as these do not change during the execution.

        Args:
            max_acc_linear (float): Linear acceleration limit in m/s^2.
            max_acc_angular (float): Angular acceleration limit in rad/s^2.
        """
        self.robot_state:RobotState = None
        self.segment_start:Pose2D = None  # (note: the segment is the line that is currently being followed)
        self.segment_end:Pose2D = None  # goal
        self.end_speed:float = None  # at goal
        self.max_speed_linear:float = None
        self.max_speed_angular:float = None
        self.max_acc_linear = max_acc_linear
        self.max_acc_angular = max_acc_angular
        self.look_at_point:Pose2D = None  # None means no look_at_point
        self.robot_angle_when_looking_at_point = None  # None if no look_at_point
    

    def update_robot_state(self, robot_state:RobotState):
        """This method should be called each time new pose and velocity of the robot are received.
        It will update the  PathFollowParams object accordingly.

        Args:
            robot_state (RobotState): New RobotState received.
        """
        self.robot_state = robot_state

    
    def update_ctrl_goal(self, robot_state:RobotState, ctrl_goal:CtrlGoal):
        """This method should be called each time a new ctrl_goal is received.
        It will update all the attributs, according to this new ctrl_goal.

        ! We also need to pass the current robot state, as the segment_start is the current pose of the robot.
        Why do we do we update segment_start here and not in update_robot_state? Because we want to keep the segment_start
        until we get a new ctrl_goal. This way, we keep the same segment to follow during the whole existence of a ctrl_goal.
        This is needed to compute the error to the segment the robot is supposed to follow. Otherwise, the segment would be
        following the robot, thus the error would always be 0.

        Args:
            robot_state (RobotState): Current state of the robot.
            ctrl_goal (CtrlGoal): New CtrlGoal received.
        """
        self.robot_state = robot_state

        self.segment_start = robot_state.pose
        self.segment_end = Pose2D(pose=ctrl_goal.pose)

        self.end_speed = ctrl_goal.end_speed
        self.max_speed_linear = ctrl_goal.max_linear_speed
        self.max_speed_angular = ctrl_goal.max_angular_speed

        if ctrl_goal.do_look_at_point:
            self.look_at_point = Pose2D(point=ctrl_goal.look_at_point)
            
        else:
            self.look_at_point = None
        self.robot_angle_when_looking_at_point = ctrl_goal.robot_angle_when_looking_at_point
    

    def __str__(self):
        return f"RobotState: {self.robot_state},\n \
                Segment Start: {self.segment_start},\n \
                Segment End: {self.segment_end},\n \
                End Speed: {self.end_speed},\n \
                Max Speed Linear: {self.max_speed_linear},\n \
                Max Speed Angular: {self.max_speed_angular},\n \
                Max Acc Linear: {self.max_acc_linear},\n \
                Max Acc Angular: {self.max_acc_angular}\n \
                Look At Point: {self.look_at_point},\n \
                Robot Angle When Looking At Point: {self.robot_angle_when_looking_at_point}"
                
