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
from std_msgs.msg import Int8, Int8MultiArray, String, Empty, Float32, Bool
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from champi_interfaces.srv import SetPose

from math import sin, cos, pi, atan2
from state_machine import ChampiStateMachine
import time
from strategies.strategy_loader import load_strategy

TOTAL_AVAILABLE_TIME = 100
# also defined in states.py
MAX_LINEAR_SPEED = 1.0 #default speed is defined in state_machine.py

class ChampiStateMachineITF(Node):

    def stm_initialized_callback(self): self.champi_sm.stm_initialized = True
    def goal_reached_callback(self): self.champi_sm.goal_reached = True

    def __init__(self):
        super().__init__('sm_ros_itf')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.get_logger().info('Launching ChampiSMRosInterface...')
        self.itf_initialized = False

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
        self.time_left = TOTAL_AVAILABLE_TIME

        self.timer = self.create_timer(timer_period_sec=0.2, callback=self.callback_timer)

        # service /set_pose
        self.client = self.create_client(SetPose, '/set_pose')


        # # Strategy
        if use_above_default_strategy_param and self.sim_param: # TODOOOOOO
            self.get_logger().info('>> Loading DEFAULT strategy...')
            self.champi_sm.color = 'YELLOW'
            self.champi_sm.strategy, self.champi_sm.init_pose, self.champi_sm.home_pose = load_strategy(get_package_share_directory('champi_brain') + '/scripts/strategies/' + strategy_file_param, self.champi_sm.color, self.get_logger())
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

        self.use_dynamic_layer_pub = self.create_publisher(Bool, '/use_dynamic_layer', 10)

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

        # subscriber topic /reset_state_machine
        self.reset_state_machine_sub = self.create_subscription(Empty, '/reset_state_machine', self.reset_state_machine_callback, 10)

        # subscriber topic /platform_distance
        self.platform_distance_sub = self.create_subscription(Float32, '/platform_distance', self.platform_distance_callback, 10)
        self.latest_platform_dist = None

        # subscriber topic /odom
        self.current_pose_sub = self.create_subscription(Odometry, '/odom', self.current_pose_callback, 10)
        self.latest_pose = None # [x, y, theta_deg]

        self.champi_sm.ros_initialized = True # TODO more things ?
        self.itf_initialized = True
        self.get_logger().warn('Launched ChampiSMRosInterface !')

    def current_pose_callback(self, msg):
        # convert from quaternion to euler angles
        theta_rad = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta_deg = theta_rad * 180.0 / pi - 90.0 # to align with the coordinate system
        self.latest_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, theta_deg]

    def reset_state_machine_callback(self, msg):
        self.champi_sm.reset()
        self.current_points = 0
        self.start_time = None
        self.time_left = TOTAL_AVAILABLE_TIME

        # Cancel current goal if self.future_navigate_result not None
        if self.goal_handle_navigate is not None:
            self.get_logger().info('Cancelling current goal...')

            future = self.goal_handle_navigate.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)

        if not self.sim_param:
            self.send_actuator_action('ENABLE_ALL_MOTORS')
            time.sleep(1)
            self.send_actuator_action('RESET_ACTUATORS')
        self.get_logger().warn('\nChampiSM has been reset!\n')

    def platform_distance_callback(self, msg):
        self.latest_platform_dist = msg.data

    def chosen_strategy_callback(self, msg):
        strategy_file_param, self.champi_sm.color = msg.data.split('#')
        self.get_logger().info(f'Chosen strategy: {strategy_file_param}, Color: {self.champi_sm.color}')

        self.get_logger().info('>> Loading strategy...')
        self.champi_sm.strategy, self.champi_sm.init_pose, self.champi_sm.home_pose = load_strategy(get_package_share_directory('champi_brain') + '/scripts/strategies/' + strategy_file_param, self.champi_sm.color, self.get_logger())
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
        self.get_logger().debug(f'Actuators finished: {array}') # TODO check for which one but should be ok without

        self.champi_sm.end_of_actuator_state = True

    def sim_user_choose_strat_and_pose(self):
        self.init_robot_pose()
        self.get_logger().info('Pose has been init')

    def callback_timer(self):
        if self.champi_sm.state == 'init_waitForTirette':
            if self.tirette_released or self.sim_param: # in sim, no tirette
                self.champi_sm.tirette_released = True
                self.start_time = self.clock.now()
                self.get_logger().warn('>> Tirette pulled! Starting match...')
                self.champi_sm.in_match = True

        if self.champi_sm.in_match:
            # CHECK TIME LEFT
            elapsed_time = (self.clock.now() - self.start_time).nanoseconds / 1e9
            self.time_left = TOTAL_AVAILABLE_TIME - elapsed_time

            if self.time_left > 0.0:
                # self.get_logger().debug(f'time left= {self.time_left}, state={self.champi_sm.state}')
                # compute approx time from robot pose to home
                dist_x_y = ((self.champi_sm.home_pose[0] - self.latest_pose[0])**2 + (self.champi_sm.home_pose[1] - self.latest_pose[1])**2)**0.5
                approx_mean_speed = MAX_LINEAR_SPEED # m/s
                approx_time_to_home = dist_x_y / approx_mean_speed + 1.0 # safety margin
                # self.get_logger().error(f'approx_time_to_home: {approx_time_to_home}s, dist={dist_x_y}')

                last_action_done = (len(self.champi_sm.strategy) == 0 and self.champi_sm.state == 'idle')
                if self.time_left <= 4.0:  # 3 last seconds, we trigger come home
                    if not self.champi_sm.state in ['comeHome', 'endOfMatch'] and not self.champi_sm.come_home_requested:
                        self.get_logger().error(f'GO HOOOOME, state={self.champi_sm.state}')

                        self.champi_sm.reset_flags()
                        self.champi_sm.come_home_requested = True
                        self.champi_sm.please_come_home() # trigger comeHome state

                elif self.time_left <= approx_time_to_home+4.0 or last_action_done:
                    if not self.champi_sm.state in ['waitToComeHome', 'comeHome', 'endOfMatch'] and not self.champi_sm.wait_to_come_home_requested:
                        self.get_logger().error(f'GO WAIT IN FRONT OF HOOOOME, state={self.champi_sm.state}, approx_time_to_home={approx_time_to_home}')

                        self.champi_sm.reset_flags()
                        self.champi_sm.wait_to_come_home_requested = True
                        self.champi_sm.please_wait_to_come_home() # trigger waitToComeHome state


            elif not self.champi_sm.match_ended:
                self.champi_sm.match_ended = True
                self.champi_sm.end_of_match()
                self.get_logger().error(f'No time left. Triggering end of match. Was in state {self.champi_sm.state}.')
                # Cancel current goal if self.future_navigate_result not None
                if self.goal_handle_navigate is not None:
                    self.get_logger().info('Cancelling current goal...')

                    future = self.goal_handle_navigate.cancel_goal_async()
                    future.add_done_callback(self.cancel_done_callback)

                    self.send_actuator_action('STOP_ALL_MOTORS')


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

        # Call service /set_pose
        request = SetPose.Request()
        request.pose = msg
        future = self.client.call_async(request)

        self.get_logger().info(f'requested set_pose to {self.champi_sm.init_pose[0]} {self.champi_sm.init_pose[1]} {self.champi_sm.init_pose[2]} rad')
        time.sleep(1)


    # ==================================== Feedback Callbacks =====================================

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug(f'Feedback received! path_compute_result:{self.path_compute_result_to_str(feedback_msg.feedback.path_compute_result)}, ETA: {round(feedback_msg.feedback.eta, 2)}s')
        # TODO prendre en compte


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
        if result.success:
            self.goal_reached_callback()
        else:
            self.get_logger().error(f'Move failed: {result.message}')
            if 'move' in self.champi_sm.state:
                self.champi_sm.cancel_current_tag()



    def cancel_done_callback(self, future):
        self.get_logger().info('Goal cancelled succesfully!')


# ============================================ Utils ==============================================
    def send_goal(self, x, y, theta_rad, use_dynamic_layer, speed, end_speed, accel_linear, accel_angular):
        self.get_logger().info(f' Call action to move to {x} {y}')

        msg = Bool()
        msg.data = use_dynamic_layer
        self.use_dynamic_layer_pub.publish(msg)


        goal_pose = Pose()
        goal_pose.position.x = x
        goal_pose.position.y = y  

        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = sin(theta_rad / 2.0)
        goal_pose.orientation.w = cos(theta_rad / 2.0)

        # Create a Navigate request and send it
        goal = self.create_action_goal(goal_pose, speed, end_speed, accel_linear, accel_angular)
        future_navigate_result = self.action_client_navigate.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future_navigate_result.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Goal sent...')


    def create_action_goal(self, goal_pose, speed, end_speed, accel_linear, accel_angular):

        goal = Navigate.Goal()
        
        goal.pose = goal_pose
        
        goal.end_speed = end_speed

        goal.max_linear_speed = speed
        goal.max_angular_speed = 3.0
        goal.accel_linear = accel_linear
        goal.accel_angular = accel_angular
        
        goal.linear_tolerance = 0.005
        goal.angular_tolerance = 0.05

        goal.do_look_at_point = False

        goal.look_at_point = Point()
        goal.look_at_point.x = 2.
        goal.look_at_point.y = 2.
        goal.look_at_point.z = 0.

        goal.robot_angle_when_looking_at_point = 0.

        goal.timeout = 20. # seconds

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
        elif action == 'STOP_ALL_MOTORS':
            msg.data = 10
        elif action == 'ENABLE_ALL_MOTORS':
            msg.data = 11
        elif action == 'GET_READY':
            msg.data = 12
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
