#!/usr/bin/env python3

import rclpy, math
from rclpy.logging import get_logger
import logging
from ament_index_python.packages import get_package_share_directory

from state_machine_custom_classes import CustomHierarchicalGraphMachine
from states import *

DEFAULT_SPEED = 0.3 # max speed is defined in itf


class ChampiStateMachine(object):
    def set_itf(self, itf): self.itf = itf
    def match_has_ended(self): return self.match_ended and self.state != 'endOfMatch'

    def __init__(self):
        self.name = 'SM'
        get_logger(self.name).info('Launching SM...')
        get_logger(self.name).set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # logging.basicConfig(level=logging.DEBUG)

        self.sm = CustomHierarchicalGraphMachine(parent = self,
                                                 name = 'Champi State Machine',
                                                 initial = StopState(name = 'stop', sm = self),
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
        self.sm.add_state(MoveForPlatformState(name='moveForPlatform', sm=self))
        self.sm.add_state(DetectPlatformState(name='detectPlatform', sm=self))
        self.sm.add_state(ChampiState(name='endOfMatch', sm=self))
        self.sm.add_state(ComeHomeState(name='comeHome', sm=self))
        self.sm.add_state(WaitToComeHomeState(name='waitToComeHome', sm=self))
        # self.sm.add_state(ChampiState(name='stop', sm=self)) # automatically created, bc it's the initial state

        self.sm.add_state(ActuatorState(name='action', sm=self))

        self.sm.get_state('init').add_substate(state=ChampiState(name='waitForRosInit', sm=self))
        # self.sm.get_state('init').add_substate(state=InitPoseState(name='moveToInitPose', sm=self))
        self.sm.get_state('init').add_substate(ChampiState(name='waitForUserChooseConfig', sm=self))
        self.sm.get_state('init').add_substate(ChampiState(name='waitForTirette', sm=self))

        # TRANSITIONS
        self.sm.add_transition('please_stop', ['init','idle','wait','move', 'moveForPlatform', 'detectPlatform','endOfMatch','action','comeHome','waitToComeHome'], 'stop', conditions='stop_requested')
        self.sm.add_transition('please_wait_to_come_home', ['init','idle','wait','move', 'moveForPlatform', 'detectPlatform', 'action'], 'waitToComeHome', conditions='wait_to_come_home_requested')
        self.sm.add_transition('please_come_home', 'waitToComeHome', 'comeHome', conditions='come_home_requested')
        ## INIT
        self.sm.add_transition('init', 'stop', 'init_waitForRosInit')
        self.sm.add_transition('init_next', 'init_waitForRosInit', 'init_waitForUserChooseConfig', conditions='ros_initialized')
        self.sm.add_transition('init_next', 'init_waitForUserChooseConfig', 'init_waitForTirette', conditions='user_has_chosen_config')
        # self.sm.add_transition('init_next', 'init_moveToInitPose', 'init_waitForTirette', conditions='goal_reached') # TODO
        self.sm.add_transition('init_end', 'init_waitForTirette', 'idle', conditions='tirette_released')
        ## BASIC ACTIONS
        self.sm.add_transition('start_move', 'idle', 'move', conditions='can_start_moving')
        self.sm.add_transition('start_detect_platform', 'idle', 'detectPlatform', conditions='can_start_detecting_platform')
        self.sm.add_transition('start_move_for_platform', 'idle', 'moveForPlatform', conditions='can_start_moving_for_platform')
        self.sm.add_transition('start_wait', 'idle', 'wait', conditions='can_start_waiting')
        self.sm.add_transition('back_to_idle', 'detectPlatform', 'idle', conditions='platformDetected')
        self.sm.add_transition('back_to_idle', ['move', 'moveForPlatform'], 'idle', conditions='goal_reached')
        self.sm.add_transition('back_to_idle', 'action', 'idle', conditions='end_of_actuator_state')
        self.sm.add_transition('back_to_idle', 'wait', 'idle', conditions='end_of_wait')
        self.sm.add_transition('end_of_match', '*', 'endOfMatch', conditions='match_has_ended')
        self.sm.add_transition('cancel_current_action', '*', 'idle', conditions='can_cancel_current_action')

        ## ACTIONS
        self.sm.add_transition('start_action', 'idle', 'action', conditions='can_start_action')
        ## POINTS
        self.sm.add_transition('add_points_done', 'idle', 'idle', conditions='add_points_is_done')

        # ADDITIONALS CALLBACKS (declared after ros_initialized so that they are not called before)
        self.sm.get_state('idle').add_callback('enter', 'find_next_state')

        # FLAGS (TO BE UPDATED BY ROS MSG CALLBACKS)
        self.reset_flags()
        self.ros_initialized = False # this one is never reset
        self.match_ended = False
        self.in_match = False
        self.come_home_requested = False
        self.wait_to_come_home_requested = False

        # STRATEGY (TO BE INIT BY THE NODE)
        self.strategy = None
        self.color = None
        self.init_pose = None
        self.home_pose = None

        # OTHERS
        self.itf:ChampiStateMachineITF = None

        self.action_list = ['PUT_BANNER',
                            'TAKE_LOWER_PLANK', 'TAKE_UPPER_PLANK',
                            'PUT_LOWER_PLANK_LAYER_1', 'PUT_UPPER_PLANK_LAYER_2',
                            'TAKE_CANS_RIGHT', 'TAKE_CANS_LEFT',
                            'PUT_CANS_LEFT_LAYER_1', 'PUT_CANS_RIGHT_LAYER_2', 'RESET_ACTUATORS', 'GET_READY']
        self.latest_canceled_tag = None # contains all the tags of the actions that have been canceled
        self.current_tag = None

        path = get_package_share_directory('champi_brain') + '/SM_diagram.png'
        get_logger(self.name).warn(f'PATH: {path}')

        self.sm.get_graph().draw(path, prog='dot')

        self.draw_graph()

        get_logger(self.name).warn('Launched SM !')
        get_logger(self.name).warn(f'Starting in state [{self.state}].')

    def reset(self):
        self.reset_flags()
        self.stop_requested = True # TODO why true ???
        self.come_home_requested = False
        self.wait_to_come_home_requested = False

        self.match_ended = False
        self.in_match = False

        self.latest_canceled_tag = None
        self.please_stop() # trigger stop state

    def reset_flags(self):
        get_logger(self.name).debug('reset flags')
        # basic flags
        self.stop_requested = False
        self.can_start_moving = False
        self.can_start_detecting_platform = False
        self.can_start_moving_for_platform = False
        self.can_start_waiting = False
        self.platformDetected = False
        self.goal_reached = False
        self.end_of_wait = False
        self.end_of_actuator_state = False
        self.add_points_is_done = False
        self.can_cancel_current_action = False

        # action flags
        self.can_start_action = False

        # msg callbacks
        # self.ros_initialized = False # in fact we don't reset it, bc once it's done it's gonna be ok
        self.user_has_chosen_config = False
        self.tirette_released = False

    def draw_graph(self, *args, **kwargs):
        #self.sm.get_combined_graph().draw(get_package_share_directory('champi_brain')+'SM_diagram.png', prog='dot')
        pass

    def cancel_current_tag(self):
        get_logger(self.name).warn(f'Canceling current action...')
        if self.current_tag is not None:
            self.latest_canceled_tag = self.current_tag
            get_logger(self.name).warn(f'All actions with tag {self.current_tag} has been canceled.')
        self.reset_flags()
        self.can_cancel_current_action = True
        self.cancel_current_action() # trigger the transition to idle state

    def find_next_state(self):
        get_logger(self.name).info(f'SM in [{self.state}], searching next action...')
        self.reset_flags()
        self.current_tag = None

        if len(self.strategy) == 0:
            get_logger(self.name).warn(f'End of actions ! Staying in [{self.state}] waiting to go home...')
            get_logger(self.name).info(f'All actions in {(100.-self.itf.time_left):.1f} seconds.')
        else:
            action = self.strategy[0]
            get_logger(self.name).info(f'Next action is {action}')

            action_name = action['action']
            get_logger(self.name).info(f' name: {action_name} & tag: {action.get('tag')}')
            if action.get('tag') is not None:
                self.current_tag = action['tag']
                if self.current_tag == self.latest_canceled_tag:
                    get_logger(self.name).warn(f'Action {action_name} with tag {self.current_tag} was previously canceled.')
                    self.strategy.pop(0)
                    self.reset_flags()
                    self.can_cancel_current_action = True
                    self.cancel_current_action() # trigger the transition to idle state
                    return
                else:
                    get_logger(self.name).debug(f'Action {action_name} with tag {self.current_tag} is valid.')

            if action_name == 'move':
                x, y, theta_deg = action['target']['x'], action['target']['y'], action['target']['theta_deg']
                speed = action['speed'] if 'speed' in action else DEFAULT_SPEED
                if 'use_dynamic_layer' in action:
                    use_dynamic_layer = action['use_dynamic_layer']
                else:
                    use_dynamic_layer = False
                self.can_start_moving = True
                self.start_move(x=x, y=y, theta_deg=theta_deg+90.0, use_dynamic_layer=use_dynamic_layer, speed=speed)  # +90° to align with the coordinate system

            elif action_name == 'moveForPlatform':
                x, y, theta_deg = action['target']['x'], action['target']['y'], action['target']['theta_deg']
                self.can_start_moving_for_platform = True
                self.start_move_for_platform(x=x, y=y, theta_deg=theta_deg)  # +90° to align with the coordinate system --> is done in the state after computations

            elif action_name == 'detectPlatform':
                x, y, theta_deg = action['target']['x'], action['target']['y'], action['target']['theta_deg']
                self.can_start_detecting_platform = True
                self.start_detect_platform(x_robot=x, y_robot=y, theta_deg_robot=theta_deg)

            elif action_name in self.action_list:
                self.can_start_action = True
                self.start_action(action=action_name)

            elif action_name == 'wait':
                self.can_start_waiting = True
                self.start_wait(duration=action['duration'])

            elif action_name == 'add_points':
                self.itf.add_points(int(action['points']))
                self.add_points_is_done = True
                self.add_points_done()

            else:
                get_logger(self.name).error('UNKNOWN ACTION')
                exit()

            self.strategy.pop(0)