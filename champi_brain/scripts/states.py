from state_machine_custom_classes import ChampiState
from rclpy.logging import get_logger
import time
from std_msgs.msg import Int8

class InitState(ChampiState):
    pass


class MoveState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)
        theta_deg = event_data.kwargs.get('theta_deg', None)
        theta_rad = theta_deg * 3.14159 / 180.0

        get_logger(self.name+'_state').info(f"Start moving to x={x}, y={y}, theta={theta_deg}°")
        self.sm.itf.send_goal(x, y, theta_rad)

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

        msg = Int8()

        if action == 'PUT_BANNER':
            msg.data = 0
        elif action == 'TAKE_LOWER_PLANK':
            msg.data = 1
        elif action == 'TAKE_UPPER_PLANK':
            msg.data = 2
        elif action == 'PUT_LOWER_PLANK_LAYER_1':
            msg.data = 3
        elif action == 'PUT_UPPER_PLANK_LAYER_2':
            msg.data = 4
        elif action == 'TAKE_CANS_FRONT':
            msg.data = 5
        elif action == 'TAKE_CANS_SIDE':
            msg.data = 6
        elif action == 'PUT_CANS_FRONT_LAYER_1':
            msg.data = 7
        elif action == 'PUT_CANS_SIDE_LAYER_2':
            msg.data = 8
        elif action == 'PUT_CANS_SIDE_LAYER_1':
            msg.data = 10


        self.sm.itf.actuators_ctrl_pub.publish(msg)

    def exit(self, event_data):
        get_logger(self.name).info(f'Action {self.action} completed')