from state_machine_custom_classes import ChampiState
from rclpy.logging import get_logger
import time

class InitState(ChampiState):
    pass


class MoveState(ChampiState):

    def enter(self, event_data):
        super().enter(event_data)

        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)
        theta_rad = event_data.kwargs.get('theta_rad', None)

        get_logger(self.name+'_state').info(f"Start moving to x={x}, y={y}, theta_rad={theta_rad}")
        self.sm.itf.send_goal(x, y, theta_rad)

class WaitState(ChampiState):
    def enter(self, event_data):
        super().enter(event_data)

        duration = event_data.kwargs.get('duration', None)

        get_logger(self.name).info(f'Waiting for {duration} seconds')
        time.sleep(duration) # TODO just a hack maybe problematic ?
        get_logger(self.name).info(f'Waited for {duration} seconds')

        self.sm.end_of_wait = True
