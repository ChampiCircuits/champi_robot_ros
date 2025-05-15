#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException

from champi_interfaces.action import Navigate
from champi_interfaces.msg import STMState
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Int8, Int8MultiArray, String
from rclpy.action import ActionClient
from math import sin, cos, pi
from state_machine import ChampiStateMachine
import time
from strategies.strategy_loader import load_strategy

TOTAL_AVAILABLE_TIME = 100


class ChampiStateMachineITF(Node):

    def stm_initialized_callback(self): self.champi_sm.stm_initialized = True
    def goal_reached_callback(self): self.champi_sm.goal_reached = True

    def tirette_pulled_callback(self): 
        self.champi_sm.tirette_pulled = True
        self.start_time = self.clock.now()
        self.get_logger().warn('>> Tirette pulled! Starting match...')

    def __init__(self):
        super().__init__('sm_ros_itf')
        self.get_logger().info('Launching ChampiSMRosInterface...')

        # Parameters
        strategy_file_param = self.declare_parameter('strategy_file', rclpy.Parameter.Type.STRING).value
        use_above_default_strategy_param = self.declare_parameter('use_above_default_strategy', rclpy.Parameter.Type.BOOL).value
        self.sim_param = self.declare_parameter('sim', rclpy.Parameter.Type.BOOL).value

        # Print parameters
        self.get_logger().info('SM interface started with the following parameters:')
        self.get_logger().info(f'strategy: {strategy_file_param}')
        self.get_logger().info(f'use_above_default_strategy_param: {use_above_default_strategy_param}')
        self.get_logger().info(f'sim: {self.sim_param}')


        self.champi_sm = ChampiStateMachine()
        self.champi_sm.set_itf(self)

        self.clock = Clock()
        self.start_time = None
        self.init_time = self.clock.now()
        self.time_left = TOTAL_AVAILABLE_TIME

        self.timer = self.create_timer(timer_period_sec=0.2, callback=self.callback_timer)

        # publisher topic /set_pose
        # Qos 'reliable' because when communicating from another computer, sometimes this msg set_pose is lost
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10  # Taille du buffer pour les messages
        )
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', qos_profile)


        # # Strategy
        if use_above_default_strategy_param:
            self.get_logger().info('>> Loading DEFAULT strategy...')
            self.champi_sm.strategy, self.champi_sm.init_pose = load_strategy(get_package_share_directory('champi_brain') + '/scripts/strategies/' + strategy_file_param, self.get_logger())
            self.get_logger().info(f'<< DEFAULT Strategy {strategy_file_param} loaded!')
            self.get_logger().info(f'<< Init pose {self.champi_sm.init_pose}')
            self.champi_sm.user_has_chosen_config = True
            self.sim_user_choose_strat_and_pose() # TODO remove

        # Action client for /navigate
        self.get_logger().info('>> Waiting for action client /navigate...')
        self.goal_handle_navigate = None
        self.action_client_navigate = ActionClient(self, Navigate, '/navigate')
        self.action_client_navigate.wait_for_server()
        self.get_logger().info('<< Action client /navigate is ready!')

        # publisher topic /ctrl/actuators
        self.actuators_ctrl_pub = self.create_publisher(Int8, '/ctrl/actuators', 10)
        if not self.sim_param:
            self.send_actuator_action('RESET_ACTUATORS')
        # subscriber topic //actuators_finished
        self.actuators_finished_sub = self.create_subscription(Int8MultiArray, '/actuators_finished', self.actuators_finished_callback, 10)
        # subscriber e_stop+tirette state
        self.STM_State_sub = self.create_subscription(STMState, '/STM_state', self.stm_state_callback, 10)
        self.e_stop_pressed = None
        self.tirette_released = None

        # publisher topic /points
        self.points_pub = self.create_publisher(Int8, '/final_score', 10)
        self.current_points = 0

        # subscriber topic /chosen_strategy
        self.chosen_strategy_sub = self.create_subscription(String, '/chosen_strategy', self.chosen_strategy_callback, 10)

        self.champi_sm.ros_initialized = True # TODO more things ?
        self.get_logger().warn('Launched ChampiSMRosInterface !')

    def chosen_strategy_callback(self, msg):
        strategy_file_param = msg.data
        self.get_logger().info(f'Chosen strategy: {strategy_file_param}')

        self.get_logger().info('>> Loading strategy...')
        self.champi_sm.strategy, self.champi_sm.init_pose = load_strategy(get_package_share_directory('champi_brain') + '/scripts/strategies/' + strategy_file_param, self.get_logger())
        self.get_logger().info(f'<< Strategy {strategy_file_param} loaded!')
        self.get_logger().info(f'<< Init pose {self.champi_sm.init_pose}')
        self.champi_sm.user_has_chosen_config = True
        self.sim_user_choose_strat_and_pose() # TODO remove

    def add_points(self, points):
        self.current_points += points

        msg = Int8()
        msg.data = self.current_points
        self.points_pub.publish(msg)

    def stm_state_callback(self, msg):
        self.e_stop_pressed = msg.e_stop_pressed
        self.tirette_released = msg.tirette_released

    def actuators_finished_callback(self, msg):
        array = msg.data # 9 elements
        self.get_logger().info(f'Actuators finished: {array}') # TODO check for which one

        self.champi_sm.end_of_actuator_state = True

    def sim_user_choose_strat_and_pose(self):
        self.init_robot_pose()
        self.get_logger().info('Pose has been init')

    def callback_timer(self):
        if not self.champi_sm.tirette_pulled:
            if self.tirette_released or self.sim_param: # in sim, no tirette
                self.tirette_pulled_callback()

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
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = self.champi_sm.init_pose[0]
        msg.pose.pose.position.y = self.champi_sm.init_pose[1]
        msg.pose.pose.position.z = 0.
        msg.pose.pose.orientation.x = 0.
        msg.pose.pose.orientation.y = 0.
        msg.pose.pose.orientation.z = sin(self.champi_sm.init_pose[2]*3.14159/180/2)
        msg.pose.pose.orientation.w = cos(self.champi_sm.init_pose[2]*3.14159/180/2)

        self.set_pose_pub.publish(msg)
        time.sleep(3)
        self.get_logger().info(f'requested set_pose to {self.champi_sm.init_pose[0]} {self.champi_sm.init_pose[1]} {self.champi_sm.init_pose[2]} rad')


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

        goal.max_linear_speed = 0.5
        goal.max_angular_speed = 3.0
        
        goal.linear_tolerance = 0.005
        goal.angular_tolerance = 0.05

        goal.do_look_at_point = False

        goal.look_at_point = Point()
        goal.look_at_point.x = 2.
        goal.look_at_point.y = 2.
        goal.look_at_point.z = 0.

        goal.robot_angle_when_looking_at_point = 0.

        goal.timeout = 15. # seconds

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


    def send_actuator_action(self, action):
        msg = Int8()

        if action == 'PUT_BANNER':
            msg.data = 0
        elif action == 'TAKE_LOWER_PLANK':
            msg.data = 1
        elif action == 'TAKE_UPPER_PLANK':
            msg.data = 2
        elif action == 'PUT_LOWER_PLANK_LAYER_1':
            msg.data = 3
        elif action == 'PUT_UPPER_PLANK_LAYER_2':
            msg.data = 4
        elif action == 'TAKE_CANS_RIGHT':
            msg.data = 5
        elif action == 'TAKE_CANS_LEFT':
            msg.data = 6
        elif action == 'PUT_CANS_RIGHT_LAYER_2':
            msg.data = 7
        elif action == 'PUT_CANS_LEFT_LAYER_1':
            msg.data = 8
        elif action == 'RESET_ACTUATORS':
            msg.data = 9
        self.actuators_ctrl_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = ChampiStateMachineITF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
