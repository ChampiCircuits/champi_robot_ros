import rclpy
from math import atan2, pi
from geometry_msgs.msg import PoseStamped
from champi_navigation.utils import dist_point_to_line, PathFollowParams

class PathHelper:
    """
    This class is used to store the path and compute the parameters needed to follow it.
    """

    def __init__(self, max_linear_speed, max_angular_speed, max_linear_acceleration, max_angular_acceleration):
        self.path = []

        self.i_goal = None
        self.current_seg_end = None
        self.current_seg_start = None

        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration

    def set_path(self, path, robot_current_state):
        if len(path) == 0:
            self.i_goal = None
            self.current_seg_end = None
            self.current_seg_start = None
            return
        
        
            # calcul de i_goal
            # on veut savoir où on se trouve sur la trajectoire, donc on regarde la distance entre la position actuelle et chaque segment du path
        segs = []
        for i in range(len(path)-1):
            p1 = path[i].pose.position
            p2 = path[i+1].pose.position
            segs.append([(p1.x, p1.y), (p2.x, p2.y)])
        
        min_dist = 100000000000000000
        min_i = 0
        for seg in segs:
            # si dist point to line
            if seg[0] != seg[1]: # division par 0
                if dist_point_to_line(robot_current_state.pose[:2], [seg[0], seg[1]]) < min_dist:
                    min_dist = dist_point_to_line(robot_current_state.pose[:2], [seg[0], seg[1]])
                    min_i = segs.index(seg)
        
        self.i_goal = min_i+1
        
        self.current_seg_end = self.pose_stamped_to_array(path[self.i_goal])
        
        if self.path == []: # This is the first path we receive
            self.current_seg_start = self.pose_stamped_to_array(path[0])
        
        elif self.is_this_a_new_path(path): # This is a new path so we need to set the start of the current segment
            self.current_seg_start = self.pose_stamped_to_array(path[self.i_goal-1])
        
        # Else, we keep the current start of the segment
        
        self.path = path


    def is_this_a_new_path(self, path):
        # The path is simply an updated version of the previous path if the second points are (almost) the same

        return not self.are_points_close(path[1], self.path[1])


    def are_points_close(self, p1, p2):
            return abs(p1.pose.position.x - p2.pose.position.x) < 0.1 and abs(p1.pose.position.y - p2.pose.position.y) < 0.1

    def pose_stamped_to_array(self, pose_stamped):
        return [pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                2*atan2(pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)]

    def robot_should_stop(self):
        return self.i_goal is None


    def update_goal(self, robot_current_state):
        if self.i_goal is None:
            return
        if self.is_current_goal_reached(robot_current_state):
            self.switch_to_next_goal()


    def is_current_goal_reached(self, robot_current_state):
        """Checks if the goal is reached and switch to the next one if it is the case.
        Should not be called if i_goal is None = no path to follow."""

        error_max_lin = 0.05
        error_max_ang = 0.05

        return (abs(robot_current_state.pose[0] - self.current_seg_end[0]) < error_max_lin
                and abs(robot_current_state.pose[1] - self.current_seg_end[1]) < error_max_lin
                and self.check_angle(robot_current_state.pose[2], self.current_seg_end[2], error_max_ang))



    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max


    def switch_to_next_goal(self):
        """If the current goal wasn't the last one, switch to the next one by incrementing i_goal.
        Otherwise, clear the cmd_path and set i_goal to None."""

        self.i_goal += 1

        if self.i_goal == len(self.path):
            self.path = []
            self.i_goal = None
            self.current_seg_start = None
            self.current_seg_end = None
            return

        self.current_seg_start = self.current_seg_end
        self.current_seg_end = self.pose_stamped_to_array(self.path[self.i_goal])


    def is_goal_the_last_one(self):
        return self.i_goal == len(self.path)-1

    def get_arrival_angle(self):
        return 2*atan2(self.path[-1].pose.orientation.z, self.path[-1].pose.orientation.w)

    def get_path_follow_params(self, robot_current_state):
        p = PathFollowParams()

        p.segment_end = self.current_seg_end
        p.segment_start = self.current_seg_start

        p.robot_state = robot_current_state

        p.arrival_angle = self.get_arrival_angle()
        p.arrival_speed = 0.3
        if self.is_goal_the_last_one():
            p.arrival_speed = 0.

        p.max_speed_linear = self.max_linear_speed
        p.max_speed_angular = self.max_angular_speed
        p.max_acc_linear = self.max_linear_acceleration
        p.max_acc_angular = self.max_angular_acceleration

        return p
