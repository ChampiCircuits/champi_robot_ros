from state_machine_custom_classes import ChampiState
from rclpy.logging import get_logger
import time
from std_msgs.msg import Int8
import math

class InitState(ChampiState):
    pass

class StopState(ChampiState):
    def enter(self, event_data):
        self.sm.stop_requested = False # request satisfied

class InitPoseState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        # TODO attendre un tag aruco OK

        x = self.sm.init_pose[0]
        y = self.sm.init_pose[1]
        theta_deg = self.sm.init_pose[2]
        theta_rad = theta_deg * 3.14159 / 180.0

        get_logger(self.name+'_state').info(f"Start moving to INIT pose: x={x}, y={y}, theta={theta_deg}°")
        self.sm.itf.send_goal(x, y, theta_rad)


class MoveState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)
        theta_deg = event_data.kwargs.get('theta_deg', None)
        use_dynamic_layer = event_data.kwargs.get('use_dynamic_layer', None)
        self.move_to(x, y, theta_deg, use_dynamic_layer)

    def move_to(self, x, y, theta_deg, use_dynamic_layer):
        theta_rad = theta_deg * math.pi / 180.0
        get_logger(self.name+'_state').info(f"Start moving to x={x}, y={y}, theta={theta_deg}°")
        self.sm.itf.send_goal(x, y, theta_rad, use_dynamic_layer)

class DetectPlatformState(ChampiState):
    def enter(self, event_data):
        """
        le robot est a une pose  X Y T  devant la plateforme
        il voit la plateforme a 30cm devant donc en X+30 Y T

        donc on peut retenir que la plateforme est a cette pose
        et les prochains moveForPlatform se basent sur ca
        """

        super().enter(event_data)
        # x,y,theta_deg are the pose of the platform in /odom frame
        x_robot = event_data.kwargs.get('x_robot', None)
        y_robot = event_data.kwargs.get('y_robot', None)
        theta_deg_robot = event_data.kwargs.get('theta_deg_robot', None)
        theta_rad_robot = theta_deg_robot * math.pi / 180.0
        get_logger(self.name).info(f'robot pose is {x_robot} {y_robot} {theta_deg_robot}°')

        get_logger(self.name).info('Starting platform detection')
        time.sleep(1)
        half_platform_width = 0.05

        # if self.sm.itf.latest_platform_dist == None and self.sm.itf.sim_param:
        #     self.sm.itf.latest_platform_dist = 0.2
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

class MoveForPlatformState(MoveState):
    def enter(self, event_data):
        # super().enter(event_data)
        # here x y theta are offsets
        x_offset = event_data.kwargs.get('x', None)
        y_offset = event_data.kwargs.get('y', None)
        theta_deg_offset = event_data.kwargs.get('theta_deg', None)
        theta_rad_offset = theta_deg_offset * math.pi / 180.0
        get_logger(self.name).info(f'offset are {x_offset} {y_offset} {theta_deg_offset}°')

        platform_center = self.sm.platform_center # theta in deg
        get_logger(self.name).info(f'platform_center is {platform_center[0]} {platform_center[1]} {platform_center[2]}°')

        # compute pose in front of platform
        # subtract the dist to the pose taking the angle in account
        # Apply rotation and translation
        x_front_platform = (x_offset * math.cos(platform_center[2]* math.pi / 180.0) - y_offset * math.sin(platform_center[2]* math.pi / 180.0)) + platform_center[0]
        y_front_platform = (x_offset * math.sin(platform_center[2]* math.pi / 180.0) + y_offset * math.cos(platform_center[2]* math.pi / 180.0)) + platform_center[1]
        theta_deg_front_platform = platform_center[2] + theta_deg_offset

        get_logger(self.name).info(f'computed pose is {x_front_platform} {y_front_platform} {theta_deg_front_platform}°')

        self.move_to(x_front_platform, y_front_platform, theta_deg_front_platform+90., use_dynamic_layer=False) # +90° to align with the coordinate system

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
        self.move_to(x, y, theta_deg, use_dynamic_layer=False)  # no dynamic_layer for home position

        self.sm.itf.add_points(10) # add 10 points for coming home, we don't wait for move to finish but flemme, should be ok ;)

class WaitToComeHomeState(MoveState):
    def enter(self, event_data):
        x = self.sm.home_pose[0]
        y = self.sm.home_pose[1] - 0.4 # to be in front of the home position
        theta_deg = self.sm.home_pose[2]

        get_logger(self.name+'_state').info(f"Start moving to WAIT FOR HOME pose: x={x}, y={y}, theta={theta_deg}°")
        self.move_to(x, y, theta_deg, use_dynamic_layer=False)  # no dynamic_layer for home position
        self.sm.itf.send_actuator_action('RESET_ACTUATORS')
