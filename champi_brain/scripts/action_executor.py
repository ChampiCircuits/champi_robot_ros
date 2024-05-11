#!/usr/bin/env python3

from robot_navigator import Robot_Navigator

import time
from typing import Tuple
from enum import Enum

from std_msgs.msg import String, ColorRGBA, Int64, Int64MultiArray, Empty

from rclpy.node import Node

import plants_taker_api
from utils import draw_rviz_markers
from utils import *
from rclpy.logging import get_logger
from icecream import ic



class Action_Executor():
    def __init__(self,strategy_node: Node) -> None:
        # check that everything is ready
        self.wait_for_ready()
        self.robot_navigator = Robot_Navigator(strategy_node)
        self.strategy_node = strategy_node

        # PLANTS STATE MACHINE
        self.state_taking_plants = None

        # PLANTS PUT STATE MACHINE
        self.plants_put = 0
        self.plants_put_pose = None

        # MAIN STATE MACHINE
        self.state = State.NO_ACTION
        self.current_action = None

        self.move_state = None
      
        self.gfini = self.strategy_node.create_subscription(Empty, '/gfini', self.gfini_callback, 10)
        

    def gfini_callback(self, msg):
        ic("GFINIIIIIIIIIIIIIIIII")
        if self.move_state == State.WAITING_END_MOVE:
            self.move_state = State.FINISHED_MOVE

        if self.state == State.JUST_MOVE:
            self.state = State.ACTION_FINISHED
        elif self.state == State.RETOUR:
            self.state = State.ACTION_FINISHED
        
        ic(self.move_state)


    def new_action(self, action:dict):
        get_logger('rclpy').info(f"ACTION EXECUTOR NEW ACTION: {action['name']}")
        if action["type"] == "prendre_plantes":
            self.state = State.PLANTES
            self.state_taking_plants = StateTakingPlants.COMPUTE_FRONT_POSE
        if action["type"] == "retour_zone":
            self.state = State.RETOUR
        if action["type"] == "pose_plantes_sol":
            self.state = State.POSE_PLANTES
        if action["type"] == "just_move":
            self.state = State.JUST_MOVE

    def update(self, current_action: dict):
        # print("update executor")
        self.current_action = current_action
        
        # get_logger('rclpy').info(f"ACTION EXECUTOR: action= {current_action['name']}")

        if self.state == State.PLANTES:
            self.update_taking_plants()
        elif self.state == State.POSE_PLANTES or self.state == State.WAITING_PLANT_PUT or self.state == State.FINISHED_PLANT_PUT:
            self.update_pose_plantes()
        # elif self.state == State.PANNEAU:
        #     self.update_panneau()
        elif self.state == State.RETOUR:
            self.update_retour()
        elif self.state == State.JUST_MOVE:
            self.update_just_move()
        
    def update_just_move(self):
        get_logger('rclpy').info(f"\Moving to to {self.current_action['pose']}")
        self.robot_navigator.navigate_to(self.current_action['pose'], 100) # TODO pas  100

    def update_retour(self):
        get_logger('rclpy').info(f"\Going home to {self.current_action['pose']}")
        self.robot_navigator.navigate_to(self.current_action['pose'], 100) # TODO pas  100
        

    def update_pose_plantes(self):
        get_logger('rclpy').info(f"\Putting plants...state={self.state}")

        if self.strategy_node.CAN_state == CAN_MSGS.RELEASE_PLANT and self.state == State.WAITING_PLANT_PUT:
            ic("FINISHED PLANT PUT")
            self.state = State.FINISHED_PLANT_PUT
        
        if self.move_state == State.FINISHED_MOVE:
            self.publish_on_CAN(CAN_MSGS.RELEASE_PLANT)
            self.plants_put += 1
            get_logger('rclpy').info(f"#########Put 1 plant out...")
            self.move_state = None
            self.state = State.WAITING_PLANT_PUT
        
        if self.strategy_node.CAN_state != CAN_MSGS.FREE and not self.state == State.WAITING_PLANT_PUT and not self.state == State.FINISHED_PLANT_PUT:
            ic("NOT FREE:",self.strategy_node.CAN_state, CAN_MSGS.FREE)
            return
        
        ic("on arive là")

        if not self.move_state == State.WAITING_END_MOVE and not self.state == State.WAITING_PLANT_PUT:
            ic("MOVING TO PUT PLANT")
            if self.plants_put < 6:
                if self.plants_put == 0:
                    self.plants_put_pose = self.current_action['pose']
                else:
                    if self.plants_put == 3:
                        get_logger('rclpy').info(f"{self.current_action['dirs'][1]}Y")
                        self.plants_put_pose = [self.plants_put_pose[0], 
                                self.plants_put_pose[1]+OFFSET_POSE_PLANT_Y*self.current_action["dirs"][1], 
                                self.plants_put_pose[2]]
                        self.robot_navigator.navigate_to(self.plants_put_pose, 10)
                        self.plants_put_pose = [self.plants_put_pose[0] - 4*OFFSET_POSE_PLANT_X*self.current_action["dirs"][0], 
                                self.plants_put_pose[1]+OFFSET_POSE_PLANT_Y*self.current_action["dirs"][1], 
                                self.plants_put_pose[2]]
                    
                    get_logger('rclpy').info(f"{self.current_action['dirs'][0]}X")
                    # get_logger('rclpy').info(f"\pose: {self.plants_put_pose}")
                    self.plants_put_pose = [self.plants_put_pose[0] + OFFSET_POSE_PLANT_X*self.current_action["dirs"][0], 
                                self.plants_put_pose[1], 
                                self.plants_put_pose[2]]
                get_logger('rclpy').info(f"\n navigate to {self.plants_put_pose}")
                self.robot_navigator.navigate_to(self.plants_put_pose, 10)
                self.move_state = State.WAITING_END_MOVE
            else:
                self.state = State.ACTION_FINISHED
                self.plants_put = 0
                self.plants_put_pose = None
                self.move_state = None
            

    def update_taking_plants(self):
        if self.state_taking_plants == StateTakingPlants.COMPUTE_FRONT_POSE:
            get_logger('rclpy').info(f"\tComputing front pose from plants")
            # compute the front pose from the plants
            get_logger('rclpy').info(f"\t\t Current pose : {self.strategy_node.robot_pose}")
            self.front_pose_from_plants = plants_taker_api.compute_front_pose_from_plants_position(self.current_action["pose"], self.strategy_node.robot_pose)
            self.state_taking_plants = StateTakingPlants.NAVIGATE_TO_FRONT_POSE
        elif self.state_taking_plants == StateTakingPlants.NAVIGATE_TO_FRONT_POSE:
            get_logger('rclpy').info(f"\tNavigating to front pose from plants: {self.front_pose_from_plants}")
            # navigate to the front pose
            # self.robot_navigator.navigate_to(self.front_pose_from_plants, self.current_action["time"])
            # self.strategy_node.robot_pose = self.front_pose_from_plants
            self.state_taking_plants = StateTakingPlants.DETECT_PLANTS
        elif self.state_taking_plants == StateTakingPlants.DETECT_PLANTS:
            get_logger('rclpy').info(f"\tDetecting plants")
            # wait for the plants to be detected
            self.plants_positions = plants_taker_api.wait_and_detect_plants(self.current_action["pose"], self.strategy_node.robot_pose)
            draw_rviz_markers(self.strategy_node, self.plants_positions, "/markers_plants_detected",ColorRGBA(r=0., g=1., b=0., a=1.), 0.06)
            draw_rviz_markers(self.strategy_node, [self.front_pose_from_plants], "/markers_selected_action",ColorRGBA(r=1., g=0., b=0., a=1.),0.05)
            self.state_taking_plants = StateTakingPlants.COMPUTE_TRAJECTORY
        elif self.state_taking_plants == StateTakingPlants.COMPUTE_TRAJECTORY:
            get_logger('rclpy').info(f"\tComputing trajectory")
            # compute the trajectory points
            self.front_pose_from_plants = (self.front_pose_from_plants[0], self.front_pose_from_plants[1], self.front_pose_from_plants[2])
            self.trajectory_points = plants_taker_api.compute_trajectory_points(self.plants_positions, self.front_pose_from_plants, self.current_action["pose"])
            self.state_taking_plants = StateTakingPlants.NAVIGATE_TO_TRAJECTORY_1
        elif self.state_taking_plants == StateTakingPlants.NAVIGATE_TO_TRAJECTORY_1:
            if not self.move_state == State.WAITING_END_MOVE and not self.move_state == State.FINISHED_MOVE:
                get_logger('rclpy').info(f"\tNavigating to first point of trajectory")
                # navigate to the first point of the trajectory
                self.robot_navigator.navigate_to(self.trajectory_points[0], self.current_action["time"])
                self.move_state = State.WAITING_END_MOVE

            if self.move_state == State.FINISHED_MOVE:
                self.publish_on_CAN(CAN_MSGS.START_GRAB_PLANTS)
                self.state_taking_plants = StateTakingPlants.NAVIGATE_TO_TRAJECTORY_2
                self.move_state = None
        elif self.state_taking_plants == StateTakingPlants.NAVIGATE_TO_TRAJECTORY_2:
            if not self.move_state == State.WAITING_END_MOVE and not self.move_state == State.FINISHED_MOVE:
                get_logger('rclpy').info(f"\tNavigating to second point of trajectory")
                # navigate to the second point of the trajectory
                self.robot_navigator.navigate_to(self.trajectory_points[1], self.current_action["time"])
                self.move_state = State.WAITING_END_MOVE
            if self.move_state == State.FINISHED_MOVE:
                self.publish_on_CAN(CAN_MSGS.STOP_GRAB_PLANTS)
                self.state_taking_plants = None
                self.state = State.NO_ACTION
                self.front_pose_from_plants = None
                self.plants_positions = None
                self.trajectory_points = None
                self.state = State.ACTION_FINISHED
                self.move_state = None



    def publish_on_CAN(self, message):
        ic("publish on CAN: ", message)
        # publish on the topic "/CAN" to be forwarded by CAN by the API
        msg = Int64()
        if message == CAN_MSGS.START_GRAB_PLANTS:
            msg.data = 0
        elif message == CAN_MSGS.STOP_GRAB_PLANTS:
            msg.data = 1
        elif message == CAN_MSGS.RELEASE_PLANT:
            msg.data = 2
        elif message == CAN_MSGS.TURN_SOLAR_PANEL:
            msg.data = 3
        self.strategy_node.CAN_pub.publish(msg)

    def wait_for_ready(self):
        # check on CAN/topics that every of the 3 stm boards are ready
        # TODO
        return True

    def stop_effectors(self):
        pass
        # self.publish_on_CAN(CAN_MSGS.)
