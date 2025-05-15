#!/usr/bin/env python3
import nav_msgs
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
from nav_msgs.msg import Odometry
from champi_interfaces.msg import STMState
from nicegui import ui

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


            ui.timer(0.5, self.check_tirette)

            self.c = 0
            print("Node created !")

        self.c += 1
        print(f"{self.c} inits of the singleton node")

    def odom_otos_callback(self, msg):
        self.last_odom_otos_time = time.time()
    def stm_state_callback(self, msg):
        self.last_stm_state = msg.data
    def check_tirette(self):
        if self.last_stm_state and self.last_stm_state.tirette_released:
            ui.open('/in_match')  # redirige vers la page
    def pub_strategy(self, strategy):
        print(f"Strategy set to {strategy}")

        msg = String()
        msg.data = strategy
        self.strategy_pub.publish(msg)







def init_ros_node() -> Node:
    if not rclpy.ok():
        rclpy.init()
    node = PagesNode()
    return node