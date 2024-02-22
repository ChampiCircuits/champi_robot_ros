from pid import PID
import avoidance 

from typing import List
from math import pi, sqrt
from icecream import ic
from skgeom import Point2, Polygon
import math_bind

class Robot_Kinematic_Model():
    def __init__(self, TABLE_WIDTH, TABLE_HEIGHT, FPS) -> None:
        self.pos = [TABLE_WIDTH/2,  # x, m
                    TABLE_HEIGHT/2,  # y, m
                    0]  # theta, rad between -pi and pi
        self.wheel_radius = 5.8
        self.wheel_width = 5
        self.robot_radius = 15
        # self.robot_positions = [self.pos]

        self.linear_speed = [0, 0]  # m/s
        self.angular_speed = 0  # rad/s
        self.has_finished_rotation = False

        self.speed_wheel0 = 0
        self.speed_wheel1 = 0
        self.speed_wheel2 = 0

        self.MAX_ACCEL_PER_CYCLE = 500/FPS # rotation/s/cycle

        self.current_goal = None
        self.goal_reached = True
        self.goals_positions = [[TABLE_WIDTH/2, TABLE_HEIGHT/2, 0],
                                ]  # [x, y, theta]
        
        self.graph = None
        self.dico_all_points = {}
        self.path_nodes = None

        self.max_ang_speed = 1.2  # pi/2  # rad/s

        # just a P
        self.pid_pos_x = PID(1, 0, 0, 1/FPS)
        self.pid_pos_y = PID(1, 0, 0, 1/FPS)
        self.pid_pos_theta = PID(1, 0, 0, 1/FPS)
        self.delta_t = 1 / FPS  # Time between two updates

    def write_speeds(self, speeds: List) -> None:
        self.speed_wheel0 = speeds[0]
        self.speed_wheel1 = speeds[1]
        self.speed_wheel2 = speeds[2]
    # TODO A CHANGER POUR LINEAR & ANGULAR 

    def update_robot_position(self):
        # DONC ICI JE FAIS LA CONVERSION,
        # ET C'EST ICI QU'IL FAUT QUE T'AJOUTES L'APPEL A LA STM ET LA LECTURE
        self.linear_to_wheel() # y'a WRITE SPEEDS dedans
        self.wheel_to_linear()

        # Update robot position
        self.pos[0] += self.linear_speed[0] * self.delta_t
        self.pos[1] += self.linear_speed[1] * self.delta_t
        self.pos[2] += self.angular_speed * self.delta_t
        # self.robot_positions.append(self.pos.copy())

    def wheel_to_linear(self):
        # Convert from WHEEL speeds --> LINEAR and ANGULAR speeds
        self.linear_speed = [-(self.speed_wheel0 + self.speed_wheel1 - 2*self.speed_wheel2),
                                 1/3*(-self.speed_wheel0*sqrt(3) + self.speed_wheel1*sqrt(3))]
        self.angular_speed = (1 / (self.robot_radius)) * \
            (-self.speed_wheel0 - self.speed_wheel1 + self.speed_wheel2)

    def linear_to_wheel(self):
        # Convert from LINEAR and ANGULAR speeds --> WHEEL speeds
        cmd_vitesse_roue0 = 0.5 * \
            self.linear_speed[0] - sqrt(3) / 2 * self.linear_speed[1] - \
            self.robot_radius * self.angular_speed
        cmd_vitesse_roue1 = 0.5 * \
            self.linear_speed[0] + sqrt(3) / 2 * self.linear_speed[1] - \
            self.robot_radius * self.angular_speed
        cmd_vitesse_roue2 = self.linear_speed[0] - \
            self.robot_radius * self.angular_speed


        # limit the speeds
        accel_roue_0 = cmd_vitesse_roue0 - self.speed_wheel0
        accel_roue_1 = cmd_vitesse_roue1 - self.speed_wheel1
        accel_roue_2 = cmd_vitesse_roue2 - self.speed_wheel2
        
        abs_accel_roue_0 = abs(accel_roue_0)
        abs_accel_roue_1 = abs(accel_roue_1)
        abs_accel_roue_2 = abs(accel_roue_2)
        abs_accel_roues = [abs_accel_roue_0, abs_accel_roue_1, abs_accel_roue_2]

        if abs_accel_roue_0 < self.MAX_ACCEL_PER_CYCLE and abs_accel_roue_1 < self.MAX_ACCEL_PER_CYCLE and abs_accel_roue_2 < self.MAX_ACCEL_PER_CYCLE:
            # acceleration requested is ok, no need to accelerate gradually.
            self.write_speeds([cmd_vitesse_roue0, cmd_vitesse_roue1, cmd_vitesse_roue2])
        else:
            speed_ratio = self.MAX_ACCEL_PER_CYCLE / max(abs_accel_roues)
            self.write_speeds([self.speed_wheel0 + speed_ratio * accel_roue_0, 
                               self.speed_wheel1 + speed_ratio * accel_roue_1, 
                               self.speed_wheel2 + speed_ratio * accel_roue_2])
            
    



class Obstacle_static_model():
    def __init__(self, center_x, center_y, width, height, offset) -> None:
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.polygon = Polygon([Point2(center_x+width/2,center_y+height/2),
                               Point2(center_x+width/2,center_y-height/2),
                               Point2(center_x-width/2,center_y-height/2),
                               Point2(center_x-width/2,center_y+height/2)])
        self.expanded_obstacle_poly = math_bind.expand(self.polygon, offset)

class Table_static_model():
    def __init__(self, width, height, offset) -> None:
        self.width = width
        self.height = height
        self.polygon = Polygon([Point2(0,0),
                                Point2(width,0),
                                Point2(width, height),
                                Point2(0, height)])
        self.expanded_poly = math_bind.expand(self.polygon, -offset)