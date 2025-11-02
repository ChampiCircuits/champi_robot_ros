from champi_brain.strategy_dsl import MotionParams
from champi_brain.state_machine_custom_classes import ChampiState
from rclpy.logging import get_logger
import time
from std_msgs.msg import Int8
import math

MAX_LINEAR_SPEED = 1.0 # also defined in itf
# TODO centralize this value in params

class InitState(ChampiState):
    pass

class StopState(ChampiState):
    def enter(self, event_data):
        self.sm.stop_requested = False # request satisfied

# class InitPoseState(ChampiState): # TODO quand on voudra init la pose du robot pendant le temps de prep automatiquement
#     def enter(self, event_data):
#         super().enter(event_data)

#         # TODO attendre un tag aruco OK

#         x = self.sm.init_pose[0]
#         y = self.sm.init_pose[1]
#         theta_deg = self.sm.init_pose[2]
#         theta_rad = theta_deg * 3.14159 / 180.0

#         get_logger(self.name+'_state').info(f"Start moving to INIT pose: x={x}, y={y}, theta={theta_deg}°")
#         # self.sm.itf.send_goal(x, y, theta_rad, use_dynamic_layer=False, speed=0.2, end_speed=0.)


class MoveState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)
        theta_deg = event_data.kwargs.get('theta_deg', None)
        motion_params = event_data.kwargs.get('motion_params', None)

        self.move_to(x, y, theta_deg, motion_params)

    def move_to(self, x, y, theta_deg, motion_params):
        theta_rad = theta_deg * math.pi / 180.0
        get_logger(self.name+'_state').info(f"Start moving to x={x}, y={y}, theta={theta_deg}° with {motion_params}")
        self.sm.itf.send_goal(x, y, theta_rad, motion_params)

class DetectPlatformState(ChampiState):
    def enter(self, event_data):
        """
        le robot est a une pose  X Y T  devant la plateforme
        il voit la plateforme a 30cm devant donc en X+30 Y T

        donc on peut retenir que la plateforme est a cette pose
        et les prochains moveForPlatform se basent sur ca
        """

    # TODO, pour l'instant cette année on utilise plus de détection interne
    # mais sinon faudra mettre à jour le world state depuis ici
    # et que le move sache qu'il doit se baser sur cette détection

        super().enter(event_data)
        # Get robot's current pose from odometry
        x_robot, y_robot, theta_deg_robot = self.sm.itf.get_current_pose()
        if x_robot is None:
            get_logger(self.name).error('Odometry not available !!')
            # stop action
            self.sm.cancel_current_tag()
            return
        
        theta_rad_robot = theta_deg_robot * math.pi / 180.0
        get_logger(self.name).info(f'robot pose is {x_robot} {y_robot} {theta_deg_robot}°')

        get_logger(self.name).info('Starting platform detection')
        time.sleep(0.5)
        half_platform_width = 0.05

        if self.sm.itf.latest_platform_dist == None and self.sm.itf.sim_param:
            self.sm.itf.latest_platform_dist = 0.2
        if self.sm.itf.latest_platform_dist < 0.0: # (pub = -1.0, but just to be sure)
            # No plank detected !! --> cancel action
            self.sm.cancel_current_tag()
        if self.sm.itf.latest_platform_dist > 0.6: # too far away, must be something else
            self.sm.cancel_current_tag()

        center_platform_dist = self.sm.itf.latest_platform_dist + half_platform_width # TODO - ??
        get_logger(self.name).info(f'Distance to platform width middle is {center_platform_dist}m')

        diff_distance = center_platform_dist

        x_front_platform = (0 * math.cos(theta_rad_robot) - diff_distance * math.sin(theta_rad_robot)) + x_robot
        y_front_platform = (0 * math.sin(theta_rad_robot) + diff_distance * math.cos(theta_rad_robot)) + y_robot
        theta_deg_front_platform = theta_deg_robot

        self.sm.platform_center = [x_front_platform, y_front_platform, theta_deg_front_platform]
        get_logger(self.name).info(f'platform pose is {x_front_platform} {y_front_platform} {theta_deg_front_platform}°')
        self.sm.platformDetected = True

class WaitState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        duration = event_data.kwargs.get('duration', None)

        get_logger(self.name).info(f'Waiting for {duration} seconds')
        time.sleep(duration)
        get_logger(self.name).info(f'Waited for {duration} seconds')

        self.sm.end_of_wait = True


class ActuatorState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        action = event_data.kwargs.get('action', None)
        self.action = action
        get_logger(self.name).info(f'Performing action: {action}')

        if self.sm.itf.sim_param:
            get_logger(self.name).info(f'Action {action} skipped in simulation mode')
            self.sm.end_of_actuator_state = True
            return

        self.sm.itf.send_actuator_action(action)

    def exit(self, event_data):
        get_logger(self.name).info(f'Action {self.action} completed')

class ComeHomeState(MoveState):
    def enter(self, event_data):
        x = self.sm.home_pose[0]
        y = self.sm.home_pose[1]
        theta_deg = self.sm.home_pose[2]

        get_logger(self.name+'_state').info(f"Start moving to HOME pose: x={x}, y={y}, theta={theta_deg}°")
        motion_params = MotionParams(
            use_dynamic_layer=False,
            speed=MAX_LINEAR_SPEED,
            end_speed=0.0,
            accel_linear=0.5,
            accel_angular=6.0
        )
        self.move_to(x, y, theta_deg, motion_params)

class WaitToComeHomeState(MoveState):
    def enter(self, event_data):
        x = self.sm.wait_to_come_home_pose[0]
        y = self.sm.wait_to_come_home_pose[1]
        theta_deg = self.sm.wait_to_come_home_pose[2]

        get_logger(self.name+'_state').info(f"Start moving to WAIT FOR HOME pose: x={x}, y={y}, theta={theta_deg}°")
        motions_params = MotionParams(
            use_dynamic_layer=True,
            speed=MAX_LINEAR_SPEED,
            end_speed=0.0,
            accel_linear=0.5,
            accel_angular=6.0
        )
        self.move_to(x, y, theta_deg, motions_params)
        self.sm.itf.send_actuator_action('RESET_ACTUATORS')
