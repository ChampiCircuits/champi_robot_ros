from state_machine_custom_classes import ChampiState
from rclpy.logging import get_logger
import time
from std_msgs.msg import Int8
import math

class InitState(ChampiState):
    pass

class InitPoseState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        # TODO attendre un scan OK

        x = self.sm.init_pose[0]
        y = self.sm.init_pose[1]
        theta_deg = self.sm.init_pose[2]
        theta_rad = theta_deg * 3.14159 / 180.0

        get_logger(self.name+'_state').info(f"Start moving to init pose: x={x}, y={y}, theta={theta_deg}°")
        self.sm.itf.send_goal(x, y, theta_rad)


class MoveState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)
        theta_deg = event_data.kwargs.get('theta_deg', None)
        self.move_to(x, y, theta_deg)

    def move_to(self, x, y, theta_deg):
        theta_rad = theta_deg * math.pi / 180.0
        get_logger(self.name+'_state').info(f"Start moving to x={x}, y={y}, theta={theta_deg}°")
        self.sm.itf.send_goal(x, y, theta_rad)

class DetectAndMoveToPlatformState(MoveState):
    def enter(self, event_data):
        # x,y,theta_deg are the pose of the platform in /odom frame
        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)
        theta_deg = event_data.kwargs.get('theta_deg', None)
        theta_rad = theta_deg * math.pi / 180.0
        get_logger(self.name).info(f'platform exact pose is {x} {y} {theta_deg}° {theta_rad} rad')

        x_robot = x
        y_robot = y # TODO
        # TODO le 0.3 devrait plutôt être la pose du robot direct
        x_robot = (0 * math.cos(theta_rad) - (-0.3) * math.sin(theta_rad)) + x_robot
        y_robot = (0 * math.sin(theta_rad) + (-0.3) * math.cos(theta_rad)) + y_robot

        theta_deg_robot = theta_deg
        get_logger(self.name).info(f'robot pose is {x_robot} {y_robot} {theta_deg_robot}°')

        get_logger(self.name).info('Starting platform detection')
        time.sleep(1)
        # TODO if is None
        half_platform_width = 0.05

        center_platform_dist = self.sm.itf.latest_platform_dist - half_platform_width
        get_logger(self.name).info(f'Distance to platform width middle is {center_platform_dist}m')

        diff_distance = (center_platform_dist - 0.21)

        # compute pose in front of platform
        # subtract the dist to the pose taking the angle in account
        # Apply rotation and translation
        x_front_platform = (0 * math.cos(theta_rad) - diff_distance * math.sin(theta_rad)) + x_robot
        y_front_platform = (0 * math.sin(theta_rad) + diff_distance * math.cos(theta_rad)) + y_robot
        theta_deg_front_platform = theta_deg_robot

        get_logger(self.name).info(f'computed pose is {x_front_platform} {y_front_platform} {theta_deg_front_platform}°')

        self.move_to(x_front_platform, y_front_platform, theta_deg_front_platform+90.) # +90° to align with the coordinate system

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