#!/usr/bin/env python3

import rclpy, math
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory

from state_machine_custom_classes import CustomHierarchicalGraphMachine
from states import ChampiState, InitState, MoveState, WaitState, ActuatorState, InitPoseState

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
        self.sm.add_state(WaitState(name='wait', sm=self))
        self.sm.add_state(MoveState(name='move', sm=self))
        self.sm.add_state(ChampiState(name='grab', sm=self))
        self.sm.add_state(ChampiState(name='put', sm=self))
        self.sm.add_state(ChampiState(name='endOfMatch', sm=self))
        # self.sm.add_state(ChampiState(name='stop', sm=self))

        self.sm.add_state(ActuatorState(name='action', sm=self))

        self.sm.get_state('init').add_substate(state=ChampiState(name='waitForRosInit', sm=self))
        self.sm.get_state('init').add_substate(state=InitPoseState(name='moveToInitPose', sm=self))
        self.sm.get_state('init').add_substate(ChampiState(name='waitForUserChooseConfig', sm=self))
        self.sm.get_state('init').add_substate(ChampiState(name='waitForTirette', sm=self))

        # TRANSITIONS
        ## INIT
        self.sm.add_transition('init', 'stop', 'init_waitForRosInit')
        self.sm.add_transition('init_next', 'init_waitForRosInit', 'init_waitForUserChooseConfig', conditions='ros_initialized')
        self.sm.add_transition('init_next', 'init_waitForUserChooseConfig', 'init_waitForTirette', conditions='user_has_choosed_config')
        # self.sm.add_transition('init_next', 'init_moveToInitPose', 'init_waitForTirette', conditions='goal_reached') # TODO
        self.sm.add_transition('init_end', 'init_waitForTirette', 'idle', conditions='tirette_pulled')
        ## BASIC ACTIONS
        self.sm.add_transition('start_move', 'idle', 'move', conditions='can_start_moving')
        self.sm.add_transition('start_wait', 'idle', 'wait', conditions='can_start_waiting')
        self.sm.add_transition('back_to_idle', 'move', 'idle', conditions='goal_reached')
        self.sm.add_transition('back_to_idle', 'action', 'idle', conditions='end_of_actuator_state')
        self.sm.add_transition('back_to_idle', 'wait', 'idle', conditions='end_of_wait')
        self.sm.add_transition('end_of_match', '*', 'endOfMatch', conditions=['match_has_ended'])
        ## ACTIONS
        self.sm.add_transition('start_action', 'idle', 'action', conditions='can_start_action')

        # ADDITIONALS CALLBACKS
        self.sm.get_state('idle').add_callback('enter', 'find_next_state')

        # FLAGS (TO BE UPDATED BY ROS MSG CALLBACKS)
        self.ros_initialized = False
        self.user_has_choosed_config = False
        self.tirette_pulled = False
        self.match_ended = False
        self.reset_flags()

        # STRATEGY (TO BE INIT BY THE NODE)
        self.strategy = None
        self.init_pose = None

        # OTHERS
        self.itf:ChampiStateMachineITF = None

        self.action_list = ['PUT_BANNER',
                            'TAKE_LOWER_PLANK', 'TAKE_UPPER_PLANK',
                            'PUT_LOWER_PLANK_LAYER_1', 'PUT_UPPER_PLANK_LAYER_2',
                            'TAKE_CANS_RIGHT', 'TAKE_CANS_LEFT',
                            'PUT_CANS_LEFT_LAYER_1', 'PUT_CANS_RIGHT_LAYER_2']

        path = get_package_share_directory('champi_brain') + '/SM_diagram.png'
        get_logger(self.name).warn(f'PATH: {path}')

        self.sm.get_graph().draw(path, prog='dot')

        self.draw_graph()

        get_logger(self.name).warn('Launched SM !')
        get_logger(self.name).warn(f'Starting in state [{self.state}].')


    def reset_flags(self):
        # basic flags
        self.can_start_moving = False
        self.can_start_waiting = False
        self.goal_reached = False
        self.end_of_wait = False
        self.end_of_actuator_state = False

        # action flags
        self.can_start_action = False

    def draw_graph(self, *args, **kwargs): 
        self.sm.get_combined_graph().draw(get_package_share_directory('champi_brain')+'SM_diagram.png', prog='dot')

    def find_next_state(self):
        get_logger(self.name).info(f'SM in [{self.state}], searching next action...')
        self.reset_flags()

        if len(self.strategy) == 0:
            get_logger(self.name).warn(f'End of actions ! Staying in [{self.state}].')
            get_logger(self.name).info(f'All actions in {100-self.itf.time_left} seconds.')
        else:
            action = self.strategy[0]
            get_logger(self.name).info(f'{action}')

            action_name = action['action']
            if action_name == 'move':
                x, y, theta_deg = action['target']['x'], action['target']['y'], action['target']['theta_deg']
                self.can_start_moving = True
                self.start_move(x=x, y=y, theta_deg=theta_deg+90.0)  # +90° to align with the coordinate system

            elif action_name in self.action_list:
                self.can_start_action = True
                self.start_action(action=action_name)

            elif action_name == 'wait':
                self.can_start_waiting = True
                self.start_wait(duration=action['duration'])
            else:
                get_logger(self.name).error('UNKNOWN ACTION')
                exit()

            self.strategy.pop(0)