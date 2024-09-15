#!/usr/bin/env python3

from robot_navigator import Robot_Navigator


from std_msgs.msg import ColorRGBA, Int64, Empty

from rclpy.node import Node

import plants_taker_api
from utils import draw_rviz_markers
from utils import State, StateTakingPlants, OFFSET_POSE_PLANT_X, OFFSET_POSE_PLANT_Y, CAN_MSGS
from rclpy.logging import get_logger



class Action_Executor():
    def __init__(self,strategy_node: Node) -> None:
        get_logger('action_exec').info(f"\tLaunching Action Executor...")

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
        self.current_action = None # TODO tout passer en Pose(), supprimer les tuples[x,y,theta]

        self.move_state = None
      
        self.gfini = self.strategy_node.create_subscription(Empty, '/gfini', self.gfini_callback, 10)

        get_logger('action_exec').info(f"\tAction Executor launched !")


    def gfini_callback(self, msg):
        get_logger('action_exec').warn(f"GFINIIIIIIIIIIIIIIIII")
        if self.move_state == State.WAITING_END_MOVE:
            self.move_state = State.FINISHED_MOVE

        if self.state == State.JUST_MOVE:
            self.state = State.ACTION_FINISHED # TODO attention, plusieurs moyens de finir JUST_MOVE et RETOUR (cf plus bas)
        elif self.state == State.RETOUR:
            self.state = State.ACTION_FINISHED
        
        get_logger('action_exec').info("\tnew move state:" + str(self.move_state))


    def new_action(self, action:dict):
        if action["type"] == "prendre_plantes":
            self.state = State.PLANTES
            self.state_taking_plants = StateTakingPlants.COMPUTE_FRONT_POSE
        if action["type"] == "retour_zone":
            self.state = State.RETOUR
        if action["type"] == "pose_plantes_sol":
            self.state = State.MOVING_BEFORE_PLANT_PUT
        if action["type"] == "just_move":
            self.state = State.JUST_MOVE

        get_logger('action_exec').warn(f"")
        get_logger('action_exec').warn(f"")
        get_logger('action_exec').warn(f"")
        get_logger('action_exec').warn(f"NEW ACTION: {action['name']} : {action['type']}")

    def update(self, current_action: dict):
        # print("update executor")
        self.current_action = current_action
        
        # get_logger('action_exec').info(f"ACTION EXECUTOR: action= {current_action['name']}")

        if self.state == State.PLANTES:
            self.update_taking_plants()
        elif self.state == State.POSE_PLANTES or self.state == State.WAITING_PLANT_PUT or self.state == State.FINISHED_PLANT_PUT or self.state == State.MOVING_BEFORE_PLANT_PUT or self.state == State.LAST_MOVE_AFTER_PUT_PLANTS:
            self.update_pose_plantes()
        # elif self.state == State.PANNEAU:
        #     self.update_panneau()
        elif self.state == State.RETOUR:
            self.update_retour()
        elif self.state == State.JUST_MOVE:
            self.update_just_move()
        
    def update_just_move(self):
        if self.move_state == State.WAITING_END_MOVE:
            return
        elif self.move_state == State.FINISHED_MOVE:
            self.state = State.ACTION_FINISHED
        get_logger('action_exec').info(f"\Moving to to {self.current_action['pose']}")
        pose = self.robot_navigator.tuple_to_pose(self.current_action['pose'])

        pose = self.robot_navigator.tuple_to_pose([1.7, 0.3, 0]) # TODO test only
        pose2 = self.robot_navigator.tuple_to_pose([1.5, 0.3, 0])
        self.robot_navigator.navigate_through_waypoints([pose, pose2], 100) # TODO pas  100
        # self.robot_navigator.navigate_through_waypoints([pose], 100) # TODO pas  100
        self.move_state = State.WAITING_END_MOVE

    def update_retour(self):
        if self.move_state == State.WAITING_END_MOVE:
            return
        elif self.move_state == State.FINISHED_MOVE:
            self.state = State.ACTION_FINISHED
        get_logger('action_exec').info(f"\Going home to {self.current_action['pose']}")
        self.robot_navigator.navigate_to_tuple(self.current_action['pose'], 100) # TODO pas  100
        self.move_state = State.WAITING_END_MOVE


    def update_pose_plantes(self):
        get_logger('action_exec').info(f"\Putting plants...state={self.state}\t nb_plants={self.strategy_node.nb_plants}\t CAN_state={self.strategy_node.CAN_state} \t MOVE_State={self.move_state}")
        
        # if self.strategy_node.IS_IN_SIM: # No CAN in simulation
        #     get_logger('action_exec').warn(f"IN SIM SO SKIPPING...")
        #     self.state = State.ACTION_FINISHED
        #     return

        """
        STATE MACHINE :
            Init: STATE = POSE_PLANTES

            s'il reste des plantes
            [
                - si STATE = POSE_PLANTES
                on va à la position suivante
                MOVE_STATE = WAITING_END_MOVE
                tant que, on return

                - si STATE c'est WAITING_PLANT_PUT et que CAN_STATE c'est RELEASE_PLANT, on a fini de poser une plante
                on met STATE à POSE_PLANTES

                - MOVE_STATE c'est devenu FINISHED_MOVE, donc on pose 1
                STATE = WAITING_PLANT_PUT
                tant que, on return
            ]
            sinon
                STATE = ACTION_FINISHED
        """
        if self.strategy_node.CAN_state == CAN_MSGS.START_GRAB_PLANTS:
            get_logger('action_exec').info(f"\twaiting end start grab plants...")
            return


        if self.state == State.LAST_MOVE_AFTER_PUT_PLANTS and self.move_state == State.FINISHED_MOVE:
            get_logger('action_exec').info(f"END OF ACTION")
            self.plants_put = 0
            self.plants_put_pose = None
            self.move_state = None
            self.publish_on_CAN(CAN_MSGS.STOP_GRAB_PLANTS) # To close the circle actuators
            self.state = State.ACTION_FINISHED
            return
        
        if self.strategy_node.nb_plants == 0:
            get_logger('action_exec').info(f"EMPTY RESERVOIR, waiting end move before switching action")
            if not self.state == State.MOVING_BEFORE_PLANT_PUT:
                get_logger('action_exec').info(f"EMPTY RESERVOIR --> last offset")
                pose = [
                    self.plants_put_pose[0] + OFFSET_POSE_PLANT_X*self.current_action["dirs"][0],
                    self.plants_put_pose[1],
                    self.plants_put_pose[2],
                ]
                self.robot_navigator.navigate_to_tuple(pose, 10)
                self.state = State.LAST_MOVE_AFTER_PUT_PLANTS
                self.move_state = State.WAITING_END_MOVE
                return

        else:
            if self.move_state == State.WAITING_END_MOVE:
                get_logger('action_exec').info(f"\twaiting end move...")
                return
            
            if self.state == State.WAITING_PLANT_PUT and self.last_plant_count_received == self.strategy_node.nb_plants + 1:
                get_logger('action_exec').warn(f"FINISHED 1 PLANT PUT")
                self.state = State.MOVING_BEFORE_PLANT_PUT
                return

            if self.state == State.WAITING_PLANT_PUT:
                get_logger('action_exec').info(f"\twaiting end plant put...")
                return
            
            if self.state == State.MOVING_BEFORE_PLANT_PUT and self.move_state == State.FINISHED_MOVE:
                # WE PUT DOWN 1 PLANT
                get_logger('action_exec').info(f"#########Putting 1 plant out...")
                self.publish_on_CAN(CAN_MSGS.RELEASE_PLANT)
                self.plants_put += 1
                self.move_state = None
                self.state = State.WAITING_PLANT_PUT
                self.last_plant_count_received = self.strategy_node.nb_plants
                return

            if self.state == State.MOVING_BEFORE_PLANT_PUT:
                # WE MOVE TO THE POSITION
                if self.plants_put == 0:
                    self.plants_put_pose = self.current_action['pose']
                else:
                    if self.plants_put == 3:
                        get_logger('action_exec').info(f"{self.current_action['dirs'][1]}Y")
                        self.plants_put_pose = [self.plants_put_pose[0], 
                                self.plants_put_pose[1]+OFFSET_POSE_PLANT_Y*self.current_action["dirs"][1], 
                                self.plants_put_pose[2]]
                        self.robot_navigator.navigate_to_tuple(self.plants_put_pose, 10)
                        self.plants_put_pose = [self.plants_put_pose[0] - 4*OFFSET_POSE_PLANT_X*self.current_action["dirs"][0], 
                                self.plants_put_pose[1]+OFFSET_POSE_PLANT_Y*self.current_action["dirs"][1], 
                                self.plants_put_pose[2]]
                    
                    get_logger('action_exec').info(f"{self.current_action['dirs'][0]}X")
                    # get_logger('action_exec').info(f"\pose: {self.plants_put_pose}")
                    self.plants_put_pose = [self.plants_put_pose[0] + OFFSET_POSE_PLANT_X*self.current_action["dirs"][0], 
                                self.plants_put_pose[1], 
                                self.plants_put_pose[2]]
                get_logger('action_exec').info(f"navigate to {self.plants_put_pose}")
                self.robot_navigator.navigate_to_tuple(self.plants_put_pose, 10)
                self.move_state = State.WAITING_END_MOVE
                self.state = State.MOVING_BEFORE_PLANT_PUT
                return
            

    def update_taking_plants(self):
        if self.state_taking_plants == StateTakingPlants.COMPUTE_FRONT_POSE:
            get_logger('action_exec').info(f"\tComputing front pose from plants")
            # compute the front pose from the plants
            get_logger('action_exec').info(f"\t\t Current pose : {self.strategy_node.robot_pose}")
            self.front_pose_from_plants = plants_taker_api.compute_front_pose_from_plants_position(self.current_action["pose"], self.strategy_node.robot_pose)
            self.state_taking_plants = StateTakingPlants.NAVIGATE_TO_FRONT_POSE
        elif self.state_taking_plants == StateTakingPlants.NAVIGATE_TO_FRONT_POSE:
            get_logger('action_exec').info(f"\tNavigating to front pose from plants: {self.front_pose_from_plants}")
            # navigate to the front pose
            # self.robot_navigator.navigate_to(self.front_pose_from_plants, self.current_action["time"])
            # self.strategy_node.robot_pose = self.front_pose_from_plants
            self.state_taking_plants = StateTakingPlants.DETECT_PLANTS
        elif self.state_taking_plants == StateTakingPlants.DETECT_PLANTS:
            get_logger('action_exec').info(f"\tDetecting plants")
            # wait for the plants to be detected
            self.plants_positions = plants_taker_api.wait_and_detect_plants(self.current_action["pose"], self.strategy_node.robot_pose)
            draw_rviz_markers(self.strategy_node, self.plants_positions, "/markers_plants_detected",ColorRGBA(r=0., g=1., b=0., a=1.), 0.06)
            draw_rviz_markers(self.strategy_node, [self.front_pose_from_plants], "/markers_selected_action",ColorRGBA(r=1., g=0., b=0., a=1.),0.05)
            self.state_taking_plants = StateTakingPlants.COMPUTE_TRAJECTORY
        elif self.state_taking_plants == StateTakingPlants.COMPUTE_TRAJECTORY:
            get_logger('action_exec').info(f"\tComputing trajectory")
            # compute the trajectory points
            self.front_pose_from_plants = (self.front_pose_from_plants[0], self.front_pose_from_plants[1], self.front_pose_from_plants[2])
            self.trajectory_points = plants_taker_api.compute_trajectory_points(self.plants_positions, self.front_pose_from_plants, self.current_action["pose"])
            self.state_taking_plants = StateTakingPlants.NAVIGATE_TO_TRAJECTORY_1
        elif self.state_taking_plants == StateTakingPlants.NAVIGATE_TO_TRAJECTORY_1:
            if not self.move_state == State.WAITING_END_MOVE and not self.move_state == State.FINISHED_MOVE:
                get_logger('action_exec').info(f"\tNavigating to first point of trajectory")
                # navigate to the first point of the trajectory
                self.robot_navigator.navigate_to_tuple(self.trajectory_points[0], self.current_action["time"])
                self.move_state = State.WAITING_END_MOVE

            if self.move_state == State.FINISHED_MOVE:
                self.publish_on_CAN(CAN_MSGS.START_GRAB_PLANTS)
                self.state_taking_plants = StateTakingPlants.NAVIGATE_TO_TRAJECTORY_2
                self.move_state = None
        elif self.state_taking_plants == StateTakingPlants.NAVIGATE_TO_TRAJECTORY_2:
            if not self.move_state == State.WAITING_END_MOVE and not self.move_state == State.FINISHED_MOVE:
                get_logger('action_exec').info(f"\tNavigating to second point of trajectory")
                # navigate to the second point of the trajectory
                self.robot_navigator.navigate_to_tuple(self.trajectory_points[1], self.current_action["time"])
                self.move_state = State.WAITING_END_MOVE
            if self.move_state == State.FINISHED_MOVE:
                get_logger('action_exec').info(f"\tEnd of taking plants")
                self.publish_on_CAN(CAN_MSGS.STOP_GRAB_PLANTS)
                self.state_taking_plants = None
                self.state = State.NO_ACTION
                self.front_pose_from_plants = None
                self.plants_positions = None
                self.trajectory_points = None
                self.state = State.ACTION_FINISHED
                self.move_state = None



    def publish_on_CAN(self, message):
        get_logger('action_exec').info(f"\tpublish on CAN: " + str(message))
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
