#!/usr/bin/env python3

from typing import Tuple
from math import acos, cos, sin
import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import rclpy.time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# markers
from std_msgs.msg import ColorRGBA

# from rclpy.executors import MultiThreadedExecutor
# from threading import Thread

import plants_taker_api
from action_executor import Action_Executor
from utils import draw_rviz_markers, calc_time_of_travel, get_action_by_name, pose_with_cov_from_position
from utils import State, StateTakingPlants

import time
from enum import Enum



actions = [
    {"name":"plantes1","type":"prendre_plantes","pose": (1.-0.3, 1.5-0.5),    "time": 10, "points": 0},
    {"name":"plantes2","type":"prendre_plantes","pose": (1.-0.5, 1.5),        "time": 10, "points": 0},
    {"name":"plantes3","type":"prendre_plantes","pose": (1.-0.3, 1.5+0.5),    "time": 10, "points": 0},
    # {"name":"plantes4","type":"prendre_plantes","pose": (1000+300, 1500-500),    "time": 10, "points": 0},
    # {"name":"plantes6","type":"prendre_plantes","pose": (1000+500, 1500),        "time": 10, "points": 0},
    # {"name":"plantes5","type":"prendre_plantes","pose": (1000+300, 1500+500),    "time": 10, "points": 0},

    {"name":"poserplantes1","type":"pose_plantes_sol","pose": (0.45/2., 0.5/2.),     "time": 10, "points": 3*6},
    {"name":"poserplantes2","type":"pose_plantes_sol","pose": (0.2-0.45/2., 0.5/2.),"time": 10, "points": 3*6},
    # {"name":"poserplantes3","type":"pose_plantes_sol","pose": (2000/2, 3000-500/2),"time": 10, "points": 3*6},
    # {"name":"poserplantes3","type":"poser_plante_jardiniere","pose": (2000/2, 3000-500/2),"time": 10, "points": 3*6},


    {"name":"panneau1","type":"tourner_panneau","pose": (0.05, 1.5-0.225),          "time": 10, "points": 5},
    # {"name":"panneau2","type":"tourner_panneau","pose": (50, 1500),              "time": 10, "points": 5},
    # {"name":"panneau3","type":"tourner_panneau","pose": (50, 1500+225),          "time": 10, "points": 5},

    # {"name":"panneau4","type":"tourner_panneau","pose": (50, 275),               "time": 10, "points": 5},
    # {"name":"panneau5","type":"tourner_panneau","pose": (50, 275+225),           "time": 10, "points": 5},
    # {"name":"panneau6","type":"tourner_panneau","pose": (50, 275+225+225),       "time": 10, "points": 5},

    # {"name":"panneau7","type":"tourner_panneau","pose": (50, 3000-275),          "time": 10, "points": 5},
    # {"name":"panneau8","type":"tourner_panneau","pose": (50, 3000-275-225),      "time": 10, "points": 5},
    # {"name":"panneau9","type":"tourner_panneau","pose": (50, 3000-275-225-225),  "time": 10, "points": 5},
    
    {"name":"retour_zone","type":"retour_zone","pose": (2.0-0.450/2., 0.5/2.),  "time": None, "points": None},

]

strategy1 = {
    "name": "strategy1",
    "init_pose": (0.45/2., 0.5/2., 0),
    "actions": {
        "list": ["plantes1", "plantes2", "plantes3", "poserplantes1", "poserplantes2", "panneau1", "retour_zone"],
    }
}

current_strategy = strategy1
TOTAL_AVAILABLE_TIME = 100 # s
ROBOT_MEAN_SPEED = 0.3 # m/s





class StrategyEngineNode(Node):
    def __init__(self) -> None:
        super().__init__('strategy_engine_node')

        self.strategy = current_strategy
        self.current_action_name = self.strategy["actions"]["list"][0]
        self.current_action_to_perform = get_action_by_name(self.current_action_name, actions)

        # STATES MACHINES
        # MAIN
        self.state = State.INIT

        
        self.start_time = None
        self.time_left = TOTAL_AVAILABLE_TIME # s, updated each iteration, considering the time to return to the zone


        init_robot_pose = self.strategy["init_pose"]
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # log init pose

        

        self.action_executor = Action_Executor(self)

        # suscribe to robot pose (x,y,theta)
        self.robot_pose = init_robot_pose
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_robot_pose, 10)

        # create /CAN publisher to publish string msgs to the CAN API
        self.CAN_publisher = self.create_publisher(String, '/CAN', 10)

        # create /stop_using_camera publisher (String which is "stop" when we want to stop using the camera and "start" when we want to start using it again)
        self.stop_cam_publisher = self.create_publisher(String, '/stop_using_camera', 10)


        self.markers_publisher_circles = self.create_publisher(MarkerArray, 'markers_selected_action', 10)
        self.markers_publisher_plants = self.create_publisher(MarkerArray, 'markers_plants_detected', 10)

        # draw init pose
        draw_rviz_markers(self, [init_robot_pose], "/markers_selected_action",ColorRGBA(r=1., g=0., b=1., a=1.),0.050)

        self.timer = self.create_timer(timer_period_sec=0.02,
                                       callback=self.callback_timer)

    def init(self):
        init_robot_pose = self.strategy["init_pose"]
        init_robot_pose = (init_robot_pose[0], init_robot_pose[1], init_robot_pose[2])

        self.pub_initial_pose.publish(pose_with_cov_from_position(init_robot_pose,self.get_clock().now().to_msg()))
        self.start_time = rclpy.time.Time()
        print("strategy engine started")

    def update_robot_pose(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, acos(msg.pose.pose.orientation.w)*2*180/3.1415)
        # print("updated robot pose:", self.robot_pose)

    def callback_timer(self):
        """Execute the strategy for one iteration"""

        if self.state == State.INIT:
            self.init()
            self.state = State.NO_ACTION

        elif self.state == State.IN_ACTION:
            self.action_executor.update(self.current_action_to_perform)
            if self.action_executor.state == State.ACTION_FINISHED:
                self.time_left = TOTAL_AVAILABLE_TIME - (rclpy.time.Time()- self.start_time).nanoseconds / 1e9
                # the action is finished, we pop it from the list
                print("Action finished:", self.current_action_name)
                self.strategy["actions"]["list"].pop(0)
                self.current_action_name = self.strategy["actions"]["list"][0]
                self.current_action_to_perform = get_action_by_name(self.current_action_name, actions)

                # print actions left
                print("Actions left:", self.strategy["actions"]["list"])
                print("*************************\n\n")

                self.state = State.NO_ACTION


        elif self.state == State.NO_ACTION:
            # projected time left is the time left minus the time to return to the zone
            projected_time_left = self.time_left - calc_time_of_travel(self.robot_pose[:2], get_action_by_name("retour_zone",actions)["pose"], ROBOT_MEAN_SPEED)

            # while the projected time left - the time to perform the current action is negative, we skip to the next action
            while projected_time_left - self.current_action_to_perform["time"] < 0:
                self.strategy["actions"]["list"].pop(0)
                self.current_action_name = self.strategy["actions"]["list"][0]
                self.current_action_to_perform = get_action_by_name(self.current_action_name)
                projected_time_left = self.time_left - calc_time_of_travel(self.robot_pose[:2], get_action_by_name("retour_zone",actions)["pose"], ROBOT_MEAN_SPEED)

            # if the time is negative we stop the effectors of the robot
            if projected_time_left < 0:
                self.action_executor.stop_effectors()
                print("STOPPING EFFECTORS")
                quit()
            # if the time is positive we execute the action
            else:
                # display selected action position
                draw_rviz_markers(self, [self.current_action_to_perform["pose"]], "/markers_selected_action",ColorRGBA(r=1., g=0., b=0., a=1.),0.250)

                print(f"Executing action {self.current_action_name}")
                self.state = State.IN_ACTION
                self.action_executor.new_action(self.current_action_to_perform)
            
        

def main(args=None):
    rclpy.init(args=args)
    # rclpy.get_logger().info("strategy engine node started")
    strategy_engine = StrategyEngineNode()

    rclpy.spin(strategy_engine)

    strategy_engine.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()