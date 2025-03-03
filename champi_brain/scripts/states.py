from state_machine_custom_classes import ChampiState

class InitState(ChampiState):
    pass


class MoveState(ChampiState):

    def enter(self, event_data):
        super().enter(event_data)

        x = event_data.kwargs.get('x', None)
        y = event_data.kwargs.get('y', None)

        print(f"Start moving to x={x}, y={y}")
        self.sm.itf.send_goal(x, y)


