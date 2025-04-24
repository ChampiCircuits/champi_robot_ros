import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from champi_interfaces.action import Navigate
from geometry_msgs.msg import Point, Pose
from rclpy.action import ActionClient
from robot_localization.srv import SetPose
from math import sin, cos, pi
from state_machine import ChampiStateMachine

TOTAL_AVAILABLE_TIME = 100


class ChampiStateMachineITF(Node):

    def stm_initialized_callback(self): self.champi_sm.stm_initialized = True
    def user_has_choosed_config_callback(self): self.champi_sm.user_has_choosed_config = True
    def goal_reached_callback(self): self.champi_sm.goal_reached = True

    def tirette_pulled_callback(self): 
        self.champi_sm.tirette_pulled = True
        self.start_time = self.clock.now()

    def __init__(self):
        super().__init__('sm_ros_itf')
        self.get_logger().info('Launching ChampiSMRosInterface...')

        # Parameters
        strategy_file_param = self.declare_parameter('strategy_file', rclpy.Parameter.Type.STRING).value
        team_color_param = self.declare_parameter('team_color', rclpy.Parameter.Type.STRING).value
            # TODO prendre en compte

        # Print parameters
        self.get_logger().info('SM interface started with the following parameters:')
        self.get_logger().info(f'strategy: {strategy_file_param}')
        self.get_logger().info(f'team_color: {team_color_param}')


        self.champi_sm = ChampiStateMachine()
        self.champi_sm.set_itf(self)

        self.clock = Clock()
        self.start_time = None
        self.init_time = self.clock.now()
        self.time_left = TOTAL_AVAILABLE_TIME

        self.timer = self.create_timer(timer_period_sec=0.2, callback=self.callback_timer)
        
       

        # # Strategy
        self.get_logger().info('>> Loading strategy...')
        self.champi_sm.strategy = self.champi_sm.load_strategy('strategies/' + strategy_file_param)
        self.get_logger().info(f'<< Strategy {strategy_file_param} loaded!')

        # Action client for /navigate
        self.goal_handle_navigate = None
        self.action_client_navigate = ActionClient(self, Navigate, '/navigate')
        self.action_client_navigate.wait_for_server()
        self.get_logger().info('Action client for /navigate is ready!')

        self.get_logger().warn('Launched ChampiSMRosInterface !')


    def sim(self):
        elapsed_time = (self.clock.now() - self.init_time).nanoseconds / 1e9

        if elapsed_time > 2.0 and not self.champi_sm.stm_initialized:
            self.stm_initialized_callback()
        if elapsed_time > 3.0 and not self.champi_sm.user_has_choosed_config:
            self.init_robot_pose()
            self.get_logger().info('Pose has been init')
            self.user_has_choosed_config_callback()
        if elapsed_time > 3.5 and not self.champi_sm.tirette_pulled:
            self.tirette_pulled_callback()

    def callback_timer(self):
        self.sim()

        in_match = self.champi_sm.tirette_pulled and self.champi_sm.state != 'end_of_match'
        if in_match:
            
            # CHECK TIME LEFT
            elapsed_time = (self.clock.now() - self.start_time).nanoseconds / 1e9
            self.time_left = TOTAL_AVAILABLE_TIME - elapsed_time

            if self.time_left > 0.0:
                pass
                # self.get_logger().info(f'time left= {self.time_left}, state={self.champi_sm.state}')
            elif not self.champi_sm.match_ended:
                self.champi_sm.match_ended = True
                self.get_logger().info(f'No time left. Triggering end of match. Was in state {self.champi_sm.state}.')

    def init_robot_pose(self):
        init_pose = [1.0, 0.5, pi/2]
        self.set_pose_client = self.create_client(SetPose, '/set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = init_pose[0]
        request.pose.pose.pose.position.y = init_pose[1]
        request.pose.pose.pose.position.z = 0.
        request.pose.pose.pose.orientation.x = 0.
        request.pose.pose.pose.orientation.y = 0.
        request.pose.pose.pose.orientation.z = sin(init_pose[2]/2)
        request.pose.pose.pose.orientation.w = cos(init_pose[2]/2)

        future = self.set_pose_client.call_async(request)


    # ==================================== Feedback Callbacks =====================================

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received! path_compute_result:{self.path_compute_result_to_str(feedback_msg.feedback.path_compute_result)}, ETA: {round(feedback_msg.feedback.eta, 2)}s')
        pass


    # ==================================== Done Callbacks ==========================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(') # TODO
            return
        
        self.goal_handle_navigate = goal_handle

        self.get_logger().info('Goal accepted!')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future): # TODO
        result = future.result().result
        self.get_logger().info(f'Action result: {result.success}, {result.message}')
        if result.success == True:
            self.goal_reached_callback()



    def cancel_done_callback(self, future):
        self.get_logger().info('Goal cancelled succesfully!') # TODO
        # self.send_goal(self.goal_pose)


# ============================================ Utils ==============================================
    def send_goal(self, x, y, theta_rad):
        self.get_logger().info(f' Call action to move to {x} {y}')
        goal_pose = Pose()
        goal_pose.position.x = x
        goal_pose.position.y = y  

        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = sin(theta_rad / 2.0)
        goal_pose.orientation.w = cos(theta_rad / 2.0)

        # Create a Navigate request and send it
        goal = self.create_action_goal(goal_pose)
        future_navigate_result = self.action_client_navigate.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future_navigate_result.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Goal sent...')


    def create_action_goal(self, goal_pose):

        goal = Navigate.Goal()
        
        goal.pose = goal_pose
        
        goal.end_speed = 0.

        goal.max_linear_speed = 0.4
        goal.max_angular_speed = 2.5
        
        goal.linear_tolerance = 0.01
        goal.angular_tolerance = 0.1

        goal.do_look_at_point = False

        goal.look_at_point = Point()
        goal.look_at_point.x = 2.
        goal.look_at_point.y = 2.
        goal.look_at_point.z = 0.

        goal.robot_angle_when_looking_at_point = 0.

        goal.timeout = 10000. # seconds

        return goal


    def path_compute_result_to_str(self, path_compute_result):
        if path_compute_result == Navigate.Feedback.SUCCESS_STRAIGHT:
            return 'SUCCESS_STRAIGHT'
        elif path_compute_result == Navigate.Feedback.SUCCESS_AVOIDANCE:
            return 'SUCCESS_AVOIDANCE'
        elif path_compute_result == Navigate.Feedback.START_NOT_IN_COSTMAP:
            return 'START_NOT_IN_COSTMAP'
        elif path_compute_result == Navigate.Feedback.GOAL_NOT_IN_COSTMAP:
            return 'GOAL_NOT_IN_COSTMAP'
        elif path_compute_result == Navigate.Feedback.GOAL_IN_OCCUPIED_CELL:
            return 'GOAL_IN_OCCUPIED_CELL'
        elif path_compute_result == Navigate.Feedback.NO_PATH_FOUND:
            return 'NO_PATH_FOUND'
        elif path_compute_result == Navigate.Feedback.INTITIALIZING:
            return 'INTITIALIZING'
        else:
            return 'UNKNOWN (error)'
        

def main(args=None):
    rclpy.init(args=args)

    ros_itf = ChampiStateMachineITF()
    rclpy.spin(ros_itf)

    ros_itf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
