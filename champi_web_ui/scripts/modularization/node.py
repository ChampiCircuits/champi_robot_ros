#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
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

            self.CAN_pub = self.create_publisher(Int64, '/act_cmd', 10)
            self.CAN_state = None
            self.nb_plants = 0

            print("Node created")
            self.c = 0
        self.c += 1
        print(self.c)


    


def init_ros_node() -> Node:
    if not rclpy.ok():
        rclpy.init()
    node = PagesNode()
    return node