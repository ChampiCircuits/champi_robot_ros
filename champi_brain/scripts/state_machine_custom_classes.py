from transitions.extensions.nesting import NestedState
from transitions.extensions import HierarchicalGraphMachine
from threading import Thread
import time
from rclpy.logging import get_logger


class ChampiState(NestedState):
    def __init__(self, sm, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sm = sm

    def enter(self, event_data):
        get_logger('sm').info(f'\033[92m>> Entering state [{self.name}]\033[0m')
        super().enter(event_data)

    def exit(self, event_data):
        get_logger('sm').info(f'\033[91m<< Exiting state [{self.name}]\033[0m')
        super().exit(event_data)


class CustomHierarchicalGraphMachine(HierarchicalGraphMachine):
    state_cls = ChampiState

    def __init__(self, parent, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.parent = parent
        self.auto_transition_thread = Thread(target=self._auto_transition_loop, daemon=True)
        self.auto_transition_thread.start()

    def _auto_transition_loop(self):
        time.sleep(2) # time for parent to init all my states, transitions and callbacks

        while True:
            for trigger in self.get_nested_triggers():
                may_trigger_method = f'may_{trigger}'

                if hasattr(self.parent, may_trigger_method):
                    may_trigger = getattr(self.parent, may_trigger_method)

                    if may_trigger():
                        trigger_method = trigger

                        if hasattr(self.parent, trigger_method):
                            trigger_action = getattr(self.parent, trigger_method)
                            trigger_action()
            
            time.sleep(0.1)