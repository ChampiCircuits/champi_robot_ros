#!/usr/bin/env python3


from champi_interfaces.msg import CtrlGoal
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.data_types.robot_state import RobotState


class PathFollowParams:
    """This class is used to store the parameters of a CmdVelUpdater class.
    We need to pass this class each time we call compute_cmd_vel, so we can change the parameters easily.
    """

    def __init__(self, max_acc_linear:float, max_acc_angular:float):
        self.robot_state:RobotState = None
        self.segment_start:Pose2D = None  # (note: the segment is the line that is currently being followed)
        self.segment_end:Pose2D = None  # goal
        self.arrival_speed:float = None  # at goal
        self.max_speed_linear:float = None
        self.max_speed_angular:float = None
        self.max_acc_linear = max_acc_linear
        self.max_acc_angular = max_acc_angular
        self.look_at_point:Pose2D = None  # None means no look_at_point

    
    def update_ctrl_goal(self, robot_state:RobotState, ctrl_goal:CtrlGoal):
        """Compute all the attributs. This method is called each time a new ctrl_goal is received.

        Args:
            robot_state (RobotState): Current state of the robot.
            ctrl_goal (CtrlGoal): New CtrlGoal received.
        """
        self.robot_state = robot_state
        self.segment_start = robot_state.pose
        self.segment_end = Pose2D(pose=ctrl_goal.pose)
        self.arrival_speed = ctrl_goal.end_speed
        self.max_speed_linear = ctrl_goal.max_linear_speed
        self.max_speed_angular = ctrl_goal.max_angular_speed
        if ctrl_goal.do_look_at_point:
            self.look_at_point = Pose2D(point=ctrl_goal.look_at_point)
        else:
            self.look_at_point = None
    
    def to_string(self):
        return f"RobotState: {self.robot_state.to_string()},\n \
                Segment Start: {self.segment_start},\n \
                Segment End: {self.segment_end},\n \
                Arrival Speed: {self.arrival_speed},\n \
                Max Speed Linear: {self.max_speed_linear},\n \
                Max Speed Angular: {self.max_speed_angular},\n \
                Max Acc Linear: {self.max_acc_linear},\n \
                Max Acc Angular: {self.max_acc_angular}\n \
                Look At Point: {self.look_at_point}\n`\n"
                
