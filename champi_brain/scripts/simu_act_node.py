#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import time

from std_msgs.msg import Int64, Int64MultiArray


from utils import CAN_MSGS

class SimuActuatorsNode(Node):

    def __init__(self) -> None:
        super().__init__('sim_actuators_node')
        get_logger('sim_actuators').warn(f"Launched actuators simu node !")

        self.act_cmd_subscription = self.create_subscription(Int64, '/act_cmd', self.act_cmd_update, 10)
        self.act_cmd_publisher = self.create_publisher(Int64MultiArray, '/act_status', 10)

        self.nb_plants = 0

        get_logger('sim_actuators').info(f"Sim actuators ready")

    def act_cmd_update(self, msg):
        get_logger('sim_actuators').info(f"Received to CAN : {msg.data}")
        if msg.data == 0:
            self.send(0)
        elif msg.data == 1:
            self.nb_plants = 6
            self.send(1)
        elif msg.data == 2:
            time.sleep(3)
            self.nb_plants -= 1
            self.send(2)

    def send(self, status):
        msg = Int64MultiArray()
        msg.data.append(status)
        msg.data.append(self.nb_plants)
        
        self.act_cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    strategy_engine = SimuActuatorsNode()

    rclpy.spin(strategy_engine)

    strategy_engine.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()