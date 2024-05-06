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
from rclpy.logging import get_logger

import time
from enum import Enum

from robot_localization.srv import SetPose


actions = [
    # {"name":"plantes1","type":"prendre_plantes","pose": (1.-0.3, 1.5-0.5),    "time": 10, "points": 0},
    # {"name":"plantes2","type":"prendre_plantes","pose": (1.-0.5, 1.5),        "time": 10, "points": 0},
    # {"name":"plantes3","type":"prendre_plantes","pose": (1.-0.3, 1.5+0.5),    "time": 10, "points": 0},
    {"name":"plantes4","type":"prendre_plantes","pose": (1.+0.3, 1.5-0.5),    "time": 10, "points": 0},
    {"name":"plantes5","type":"prendre_plantes","pose": (1.+0.5, 1.5),        "time": 10, "points": 0},
    {"name":"plantes6","type":"prendre_plantes","pose": (1.+0.3, 1.5+0.5),    "time": 10, "points": 0},

    {"name":"poserplantesJ1","type":"pose_plantes_sol","pose": (0.45/2., 0.5/2, -1.0466),     "time": 10, "points": 3*6, "dirs": (1,1)},
    {"name":"poserplantesJ2","type":"pose_plantes_sol","pose": (2.0-0.45, 0.5, -1.0466),"time": 10, "points": 3*6, "dirs":(1,1)},
    {"name":"poserplantesJ3","type":"pose_plantes_sol","pose": (1.0, 3.0-0.5/2, -1.0466+3.14),"time": 10, "points": 3*6, "dirs":(-1, -1)},
    
    {"name":"poserplantesB1","type":"pose_plantes_sol","pose": (1.0, 0.5/2, -1.0466),"time": 10, "points": 3*6, "dirs":(1, 1)},
    {"name":"poserplantesB2","type":"pose_plantes_sol","pose": (0.45/2, 3.0-0.4/2, -1.0466),"time": 10, "points": 3*6, "dirs":(1, -1)},
    {"name":"poserplantesB3","type":"pose_plantes_sol","pose": (2.0-0.45/2.0, 3.0-0.5/2, -1.0466+3.14),"time": 10, "points": 3*6, "dirs":(-1, -1)},


    # {"name":"poserplantes3","type":"poser_plante_jardiniere","pose": (2000/2, 3000-500/2),"time": 10, "points": 3*6},


    # {"name":"panneau1","type":"tourner_panneau","pose": (0.05, 1.5-0.225),          "time": 10, "points": 5},
    # {"name":"panneau2","type":"tourner_panneau","pose": (50, 1500),              "time": 10, "points": 5},
    # {"name":"panneau3","type":"tourner_panneau","pose": (50, 1500+225),          "time": 10, "points": 5},

    # {"name":"panneau4","type":"tourner_panneau","pose": (50, 275),               "time": 10, "points": 5},
    # {"name":"panneau5","type":"tourner_panneau","pose": (50, 275+225),           "time": 10, "points": 5},
    # {"name":"panneau6","type":"tourner_panneau","pose": (50, 275+225+225),       "time": 10, "points": 5},

    # {"name":"panneau7","type":"tourner_panneau","pose": (50, 3000-275),          "time": 10, "points": 5},
    # {"name":"panneau8","type":"tourner_panneau","pose": (50, 3000-275-225),      "time": 10, "points": 5},
    # {"name":"panneau9","type":"tourner_panneau","pose": (50, 3000-275-225-225),  "time": 10, "points": 5},
    
    {"name":"retour_zone_yellow","type":"retour_zone","pose": (1.0, 2.734, 3.14),  "time": 0, "points": None}, #J3
    {"name":"retour_zone_blue","type":"retour_zone","pose": (1.0, 0.265, 1.57),  "time": 0, "points": None}, #B1
    {"name":"move_middle","type":"just_move","pose": (1.0, 1.5, 1.57),  "time": 0, "points": None},
    {"name":"move_test","type":"just_move","pose": (1.5, 2.0, -1.57),  "time": 0, "points": None},

]
# ZONE JAUNE BAS GAUCHE 0.145, 0.165, 0.5233
# ZONE JAUNE BAS DROITE 1.855, 0.165, 2.6166
# ZONE JAUNE HAUT MILIEU 1.0, 2.834, 3.14
# init_pose_yellow = [1, 1., 0.0]
# init_pose_yellow = [1.855, 0.165, 2.6166]

strategy_yellow = {
    "name": "strategy_yellow",
    "init_pose": (1.855, 0.165, 2.6166),
    "actions": {
        # "list": ["poserplantesJ2"]
        # "list": ["plantes4","retour_zone_yellow"],
        "list": ["plantes4","poserplantesJ2","plantes5","poserplantesJ1","plantes6","poserplantesJ3", "retour_zone_yellow"],
        # "list": ["plantes4", "plantes5", "plantes6", "poserplantes1", "poserplantes2", "panneau1", "retour_zone_yellow"],
    }
}
strategy_blue = {
    "name": "strategy_blue",
    "init_pose": (1.855, 2.835, 4.1866),
    "actions": {
        "list": ["plantes6","poserplantesB3","plantes5","poserplantesB2","plantes4","poserplantesB1", "retour_zone_blue"],
        # "list": ["plantes4","retour_zone_blue"],
        "list": ["plantes4","poserplantesB1", "retour_zone_blue"],
        # "list": ["plantes4", "plantes5", "plantes6", "poserplantes1", "poserplantes2", "panneau1", "retour_zone_blue"],
    }
}

TOTAL_AVAILABLE_TIME = 100 # s
ROBOT_MEAN_SPEED = 0.3 # m/s


class StrategyEngineNode(Node):
    def __init__(self) -> None:
        super().__init__('strategy_engine_node')

        # depending on the ros2 parameter 'color'
        global TEAM
        global RETOUR_ZONE_NAME
        self.declare_parameter('color', rclpy.Parameter.Type.STRING) 

        if self.get_parameter('color').get_parameter_value().string_value == 'yellow':
            TEAM = "YELLOW"
            self.get_logger().info("YELLOW")
        else:
            TEAM = "BLUE"
            self.get_logger().info("BLUE")
        self.get_logger().info("TEAM: "+TEAM)
        if TEAM == "YELLOW":
            current_strategy = strategy_yellow
            RETOUR_ZONE_NAME = "retour_zone_yellow"
        else:
            current_strategy = strategy_blue
            RETOUR_ZONE_NAME = "retour_zone_blue"

        self.get_logger().info("\n\n\n\n")
        self.get_logger().info(self.get_parameter('color').get_parameter_value().string_value)
        self.get_logger().info("\n\n\n\n")

        self.strategy = current_strategy
        self.current_action_name = self.strategy["actions"]["list"][0]
        self.current_action_to_perform = get_action_by_name(self.current_action_name, actions)

        # STATES MACHINES
        # MAIN
        self.state = State.INIT


        self.start_time = None
        self.time_left = TOTAL_AVAILABLE_TIME # s, updated each iteration, considering the time to return to the zone


        init_robot_pose = self.strategy["init_pose"]
        # self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # call service set_pose (0.145,0.165), theta = 30°
        self.set_pose_client = self.create_client(SetPose, '/set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = init_robot_pose[0]
        request.pose.pose.pose.position.y = init_robot_pose[1]
        request.pose.pose.pose.position.z = 0.
        request.pose.pose.pose.orientation.x = 0.
        request.pose.pose.pose.orientation.y = 0.
        request.pose.pose.pose.orientation.z = sin(init_robot_pose[2]/2)
        request.pose.pose.pose.orientation.w = cos(init_robot_pose[2]/2)

        future = self.set_pose_client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    break
                else:
                    self.get_logger().info('service call failed %r' % (future.exception(),))
                    break

        
        time.sleep(3)
        self.action_executor = Action_Executor(self)

        # suscribe to robot pose (x,y,theta)
        self.robot_pose = init_robot_pose
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.update_robot_pose, 10)

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
        # init_robot_pose = self.strategy["init_pose"]
        # init_robot_pose = (init_robot_pose[0], init_robot_pose[1], init_robot_pose[2])

        # self.pub_initial_pose.publish(pose_with_cov_from_position(init_robot_pose,self.get_clock().now().to_msg()))
        self.start_time = rclpy.time.Time()
        get_logger('rclpy').info(f"strategy engine started")

    def update_robot_pose(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, acos(msg.pose.pose.orientation.w)*2*180/3.1415)
        # get_logger('rclpy').info(f"updated robot_pose: {self.robot_pose}")

    def callback_timer(self):
        """Execute the strategy for one iteration"""

        if self.state == State.INIT:
            self.init()
            self.state = State.NO_ACTION

        elif self.state == State.IN_ACTION:
            self.action_executor.update(self.current_action_to_perform)

            if self.action_executor.state == State.ACTION_FINISHED:
                get_logger('rclpy').info(f"")
                get_logger('rclpy').info(f"Action finished: {self.current_action_name}")
                get_logger('rclpy').info(f"actions dispos: {self.strategy['actions']['list']}")
                self.action_executor.state = State.NO_ACTION

                # if the last action executed was "retour_zone" then we stop everything
                if self.current_action_to_perform == "retour_zone":
                    quit()

                # self.time_left = TOTAL_AVAILABLE_TIME - (rclpy.time.Time()- self.start_time).nanoseconds / 1e9
                # # the action is finished, we pop it from the list
                # get_logger('rclpy').info(f"Action finished: {self.current_action_name}")
                # get_logger('rclpy').info(f"actions dispos: {self.strategy['actions']['list']}")

                # if len(self.strategy["actions"]["list"]) != 0:
                #     self.strategy["actions"]["list"].pop(0)
                #     self.current_action_name = self.strategy["actions"]["list"][0]
                #     self.current_action_to_perform = get_action_by_name(self.current_action_name, actions)

                #     # print actions left
                #     get_logger('rclpy').info(f"Actions left: {self.strategy['actions']['list']}")
                #     get_logger('rclpy').info(f"*************************\n\n")
                # else:
                #     get_logger('rclpy').info(f"No action left")
                #     get_logger('rclpy').info(f"*************************\n\n")



                self.state = State.NO_ACTION


        elif self.state == State.NO_ACTION:
            # projected time left is the time left minus the time to return to the zone
            projected_time_left = self.time_left - calc_time_of_travel(self.robot_pose[:2], get_action_by_name(RETOUR_ZONE_NAME,actions)["pose"], ROBOT_MEAN_SPEED)
            
            # while the projected time left - the time to perform the current action is negative, we skip to the next action
            # while projected_time_left - self.current_action_to_perform["time"] < 0:
            #     self.strategy["actions"]["list"].pop(0) # TODO verif que on a encore des actions dispos
            #     self.current_action_name = self.strategy["actions"]["list"][0]
            #     self.current_action_to_perform = get_action_by_name(self.current_action_name)
            #     projected_time_left = self.time_left - calc_time_of_travel(self.robot_pose[:2], get_action_by_name("retour_zone",actions)["pose"], ROBOT_MEAN_SPEED)

            # if the time is negative we stop the effectors of the robot
            if projected_time_left < 0:
                self.action_executor.stop_effectors()
                get_logger('rclpy').info(f"STOPPING EFFECTORS")
                quit()
            # if the time is positive we execute the action
            else:
                # display selected action position
                draw_rviz_markers(self, [self.current_action_to_perform["pose"]], "/markers_selected_action",ColorRGBA(r=1., g=0., b=0., a=1.),0.250)
                if len(self.strategy["actions"]["list"]) != 0:
                    self.current_action_name = self.strategy["actions"]["list"][0]
                    self.current_action_to_perform = get_action_by_name(self.current_action_name, actions)
                    self.strategy["actions"]["list"].pop(0) # TODO verif que on a encore des actions dispos
                    get_logger('rclpy').info(f"")
                    get_logger('rclpy').info(f"Executing action {self.current_action_name}")
                    get_logger('rclpy').info(f"Actions left {self.strategy['actions']['list']}")

                    self.state = State.IN_ACTION
                    self.action_executor.new_action(self.current_action_to_perform)
                else:
                    get_logger('rclpy').info(f"FIN")
                    quit()
            
        

def main(args=None):
    rclpy.init(args=args)
    strategy_engine = StrategyEngineNode()

    rclpy.spin(strategy_engine)

    strategy_engine.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()