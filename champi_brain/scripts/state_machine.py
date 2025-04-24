#!/usr/bin/env python3

import logging, rclpy, typing, time
from rclpy.logging import get_logger

from state_machine_custom_classes import CustomHierarchicalGraphMachine
from state_machine_itf import ChampiStateMachineITF
from states import ChampiState, InitState, MoveState

strategy = ['move 1 1', 'grab', 'move 1 2', 'put']


class ChampiStateMachine(object):
    def set_itf(self, itf): self.itf = itf
    def match_has_ended(self): return self.match_ended and self.state != 'endOfMatch'

    def __init__(self):
        self.name = 'SM'
        get_logger(self.name).info('Launching SM...')
        # logging.basicConfig(level=logging.INFO)


        self.sm = CustomHierarchicalGraphMachine(parent = self,
                                                 name = 'Champi State Machine',
                                                 initial = ChampiState(name = 'stop', sm = self),
                                                 model = self,
                                                 graph_engine = 'graphviz',
                                                 queued = True,
                                                 auto_transitions = False,
                                                 before_state_change = 'draw_graph',
                                                 after_state_change = 'draw_graph')

        # STATES
        # !! States names must not contain underscores _ as they are used for substates naming
        self.sm.add_state(InitState(name='init', sm=self))
        self.sm.add_state(ChampiState(name='idle', sm=self))
        self.sm.add_state(MoveState(name='move', sm=self))
        self.sm.add_state(ChampiState(name='grab', sm=self))
        self.sm.add_state(ChampiState(name='put', sm=self))
        self.sm.add_state(ChampiState(name='endOfMatch', sm=self))
        # self.sm.add_state(ChampiState(name='stop', sm=self))

        self.sm.get_state('init').add_substate(state=ChampiState(name='waitForStmInit', sm=self))
        self.sm.get_state('init').add_substate(ChampiState(name='waitForUserChooseConfig', sm=self))
        self.sm.get_state('init').add_substate(ChampiState(name='waitForTirette', sm=self))

        # TRANSITIONS
        self.sm.add_transition('init', 'stop', 'init_waitForStmInit')
        self.sm.add_transition('init_next', 'init_waitForStmInit', 'init_waitForUserChooseConfig', conditions='stm_initialized')
        self.sm.add_transition('init_next', 'init_waitForUserChooseConfig', 'init_waitForTirette', conditions='user_has_choosed_config')
        self.sm.add_transition('init_end', 'init_waitForTirette', 'idle', conditions='tirette_pulled')

        self.sm.add_transition('start_move', 'idle', 'move', conditions='can_start_moving')
        self.sm.add_transition('start_grab', 'idle', 'grab', conditions='can_start_grabbing')
        self.sm.add_transition('start_put', 'idle', 'put', conditions='can_start_putting')
        self.sm.add_transition('back_to_idle', 'move', 'idle', conditions='goal_reached')
        self.sm.add_transition('back_to_idle', ['grab', 'put'], 'idle')
        self.sm.add_transition('end_of_match', '*', 'endOfMatch', conditions=['match_has_ended'])

        # ADDITIONALS CALLBACKS
        self.sm.get_state('idle').add_callback('enter', 'find_next_state')

        # FLAGS TO BE UPDATED BY ROS MSG CALLBACKS
        self.stm_initialized = False
        self.user_has_choosed_config = False
        self.tirette_pulled = False
        self.match_ended = False
        self.reset_flags()


        # OTHERS
        self.itf:ChampiStateMachineITF = None
        self.sm.get_graph().draw('SM_diagram.png', prog='dot')
        get_logger(self.name).warn('Launched SM !')
        get_logger(self.name).warn(f'Starting in state [{self.state}].')

    def reset_flags(self):
        self.can_start_moving = False
        self.can_start_grabbing = False
        self.can_start_putting = False
        self.goal_reached = False

    def draw_graph(self, *args, **kwargs): 
        self.sm.get_combined_graph().draw('SM_diagram.png', prog='dot')

    def find_next_state(self):
        get_logger(self.name).info(f'SM in [{self.state}], searching next action...')
        self.reset_flags()

        if len(strategy) == 0:
            get_logger(self.name).warn(f'End of actions ! Staying in [{self.state}].')
        else:
            action = strategy[0]
            action_name = action.split(' ')[0]
            if action_name == 'move':
                x, y, theta_rad = action['target']['x'], action['target']['y'], action['target']['theta_rad']
                self.can_start_moving = True
                self.start_move(x=x, y=y, theta_rad=theta_rad)

            elif action_name == 'grab':
                self.can_start_grabbing = True
                self.start_grab()
            elif action_name == 'put':
                self.can_start_putting = True
                self.start_put()
            else:
                get_logger(self.name).error('UNKNOWN ACTION')
                exit()

            strategy.pop(0)



def main(args=None):
    rclpy.init(args=args)

    sm = ChampiStateMachine()
    ros_itf = ChampiStateMachineITF(sm)
    rclpy.spin(ros_itf)

    ros_itf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

