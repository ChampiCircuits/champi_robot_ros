#!/usr/bin/env python3
import nav_msgs
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
from nav_msgs.msg import Odometry
from champi_interfaces.msg import STMState
from std_msgs.msg import Int8, Empty

from enum import Enum
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String


class PagesNode(Node):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(PagesNode, cls).__new__(cls, *args, **kwargs)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_initialized'):  # Pour éviter la réinitialisation
            super().__init__('GUI_diagnostics_node')
            self._initialized = True


            self.sub_odom_otos = self.create_subscription(
                nav_msgs.msg.Odometry,
                '/odom_otos',
                self.odom_otos_callback,
                10)
            self.last_odom_otos_time = -1

            self.sub_stm_state = self.create_subscription(
                STMState,
                '/STM_state',
                self.stm_state_callback,
                10)
            self.last_stm_state:STMState = None

            self.strategy_pub = self.create_publisher(
                String,
                '/chosen_strategy',
                10
            )

            self.score_subscriber = self.create_subscription(Int8, '/final_score', self.update_score, 10)

            # Variables used during match
            self.score = 0
            self.time_left = 100
            self.start_time = None
            self.match_started = False
            self.ready_to_start_match = False

            self.timer = self.create_timer(0.1, self.update)

            self.reset_state_machine_pub =  self.create_publisher(
                Empty,
                '/reset_state_machine',
                10
            )

            self.c = 0
            self.get_logger().info("Node created !")

        self.c += 1
        self.get_logger().info(f"{self.c} inits of the singleton node")

    def update(self):
        if not self.match_started:
            if self.last_stm_state is not None and self.last_stm_state.tirette_released and self.ready_to_start_match:
                self.get_logger().info("\n\n!! MATCH STARTED !!\n\n")

                self.score = 0
                self.time_left = 100
                self.match_started = True
                self.start_time = time.time()


    def reset_all(self):
        self.get_logger().warn('Node has been reset !')
        self.score = 0
        self.time_left = 100
        self.start_time = None
        self.match_started = False
        self.ready_to_start_match = False

        self.get_logger().warn('Resetting the state machine !')
        self.reset_state_machine_pub.publish(Empty())

    def update_score(self, received_score: Int8):
        self.score = received_score.data
        self.get_logger().info(f'received score : {received_score}')
    def odom_otos_callback(self, msg):
        self.last_odom_otos_time = time.time()
    def stm_state_callback(self, msg):
        self.last_stm_state = msg

    def pub_strategy(self, strategy):
        self.get_logger().info(f'Strategy set to {strategy}')

        msg = String()
        msg.data = strategy
        self.strategy_pub.publish(msg)




def init_ros_node() -> Node:
    if not rclpy.ok():
        rclpy.init()
    node = PagesNode()
    return node