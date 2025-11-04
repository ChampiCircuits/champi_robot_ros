#!/usr/bin/env python3
"""
New State Machine - Pure Business Logic
No ROS dependencies - uses ActionExecutor interface
"""

import math
import time
from typing import List, Optional, Callable
from dataclasses import dataclass

from champi_brain.core.action_executor import ActionExecutor
from champi_brain.core.match_controller import MatchController
from champi_brain.strategy_dsl import Action, MotionParams, Position
from rclpy.logging import get_logger


@dataclass
class StateMachineConfig:
    """Configuration for the state machine."""
    color: str  # 'blue' or 'yellow'
    init_pose: tuple[float, float, float]  # (x, y, theta_deg)
    home_pose: tuple[float, float, float]  # (x, y, theta_deg)
    wait_to_come_home_pose: tuple[float, float, float]  # (x, y, theta_deg)
    simulation_mode: bool = False


class StateMachine:
    """
    Pure state machine logic without ROS dependencies.
    Uses ActionExecutor interface for actions.
    """
    
    # State definitions
    STATE_STOP = 'stop'
    STATE_INIT = 'init'
    STATE_IDLE = 'idle'
    STATE_EXECUTING_ACTION = 'executing_action'
    STATE_WAIT_TO_COME_HOME = 'wait_to_come_home'
    STATE_COME_HOME = 'come_home'
    STATE_END_OF_MATCH = 'end_of_match'
    
    def __init__(
        self,
        executor: ActionExecutor,
        match_controller: MatchController,
        config: StateMachineConfig
    ):
        """
        Initialize the state machine.
        
        Args:
            executor: ActionExecutor implementation for robot actions
            match_controller: MatchController for timing and scoring
            config: Configuration parameters
        """
        self.executor = executor
        self.match = match_controller
        self.config = config
        self.logger = get_logger('state_machine')
        
        # State
        self.state = self.STATE_STOP
        self.strategy: List[Action] = []
        self.current_action: Optional[Action] = None
        self.current_tag: Optional[str] = None
        self.canceled_tags: set[str] = set()
        
        # Flags for state transitions
        self._stop_requested = False
        self._action_completed = False
        self._platform_detected = False
        self.platform_center: Optional[tuple[float, float, float]] = None
        
        # Initialization flags
        self._ros_initialized = False
        self._config_chosen = False
        self._tirette_released = False
        
        # Callbacks for external events
        self.on_state_changed: Optional[Callable[[str], None]] = None
        self.on_strategy_completed: Optional[Callable[[], None]] = None
        
    # =================================================================
    # PUBLIC API
    # =================================================================
    
    def set_strategy(self, strategy: List[Action]) -> None:
        """Set the strategy (list of actions) to execute."""
        self.strategy = strategy.copy()
        
    def start_initialization(self) -> None:
        """Start the initialization sequence."""
        self._transition_to(self.STATE_INIT)
        
    def notify_ros_initialized(self) -> None:
        """Notify that ROS is ready."""
        self._ros_initialized = True
        self._check_init_progress()
        
    def notify_config_chosen(self) -> None:
        """Notify that user has chosen configuration."""
        self._config_chosen = True
        self._check_init_progress()
        
    def notify_tirette_released(self) -> None:
        """Notify that start button/tirette is released."""
        self._tirette_released = True
        self._check_init_progress()
        
    def request_stop(self) -> None:
        """Request emergency stop."""
        self._stop_requested = True
        self._transition_to(self.STATE_STOP)
        
    def request_come_home(self) -> None:
        """Request to return home immediately."""
        if self.state != self.STATE_STOP and self.state != self.STATE_END_OF_MATCH:
            self._cancel_current_action()
            self._transition_to(self.STATE_COME_HOME)
            
    def request_wait_to_come_home(self) -> None:
        """Request to move to waiting position before coming home."""
        if self.state == self.STATE_EXECUTING_ACTION or self.state == self.STATE_IDLE:
            self._cancel_current_action()
            self._transition_to(self.STATE_WAIT_TO_COME_HOME)
            
    def notify_action_completed(self) -> None:
        """Notify that current action is completed."""
        self._action_completed = True
        if self.state == self.STATE_EXECUTING_ACTION:
            self._transition_to(self.STATE_IDLE)
            
    def notify_platform_detected(self, platform_pose: tuple[float, float, float]) -> None:
        """Notify that platform has been detected."""
        self.platform_center = platform_pose
        self._platform_detected = True
        self.notify_action_completed()
        
    def cancel_current_tag(self) -> None:
        """Cancel all actions with the current tag."""
        if self.current_tag:
            self.canceled_tags.add(self.current_tag)
        self._cancel_current_action()
        # Transition back to idle after canceling
        if self.state == self.STATE_EXECUTING_ACTION:
            self._transition_to(self.STATE_IDLE)
        
    def update(self) -> None:
        """
        Main update loop - call periodically.
        Handles state transitions and action execution.
        """
        # Check if match has ended
        if self.match.is_match_started() and self.match.get_remaining_time() <= 0:
            if self.state != self.STATE_END_OF_MATCH:
                self._cancel_current_action()
                self._transition_to(self.STATE_END_OF_MATCH)
                return
        
        # Handle state-specific logic
        if self.state == self.STATE_IDLE:
            self._handle_idle_state()
    
    # =================================================================
    # INTERNAL STATE MACHINE LOGIC
    # =================================================================
    
    def _transition_to(self, new_state: str) -> None:
        """Transition to a new state."""
        if self.state == new_state:
            return
            
        old_state = self.state
        self.state = new_state
        
        self.logger.debug(f"[SM] Transition: {old_state} -> {new_state}")
        
        # State entry actions
        if new_state == self.STATE_IDLE:
            self._on_enter_idle()
        elif new_state == self.STATE_COME_HOME:
            self._on_enter_come_home()
        elif new_state == self.STATE_WAIT_TO_COME_HOME:
            self._on_enter_wait_to_come_home()
        elif new_state == self.STATE_END_OF_MATCH:
            self._on_enter_end_of_match()
            
        # Notify observers
        if self.on_state_changed:
            self.on_state_changed(new_state)
    
    def _check_init_progress(self) -> None:
        """Check initialization progress and advance if ready."""
        if not self._ros_initialized:
            self.logger.info("[SM] Waiting for ROS initialization...")
            return
            
        if not self._config_chosen:
            self.logger.info("[SM] Waiting for user to choose configuration...")
            return
            
        if not self._tirette_released:
            self.logger.info("[SM] Waiting for tirette release...")
            return
            
        # All initialization steps complete - start match
        self.logger.info("[SM] Initialization complete! Starting match...")
        self.match.start_match()
        self._transition_to(self.STATE_IDLE)
    
    def _on_enter_idle(self) -> None:
        """Handle entry into idle state - find next action."""
        self.current_action = None
        self._action_completed = False
        self._platform_detected = False
        
        # Process next action from strategy
        self._find_next_action()
    
    def _on_enter_come_home(self) -> None:
        """Handle entry into come home state."""
        x, y, theta_deg = self.config.home_pose
        motion = MotionParams(
            speed=1.0,
            end_speed=0.0,
            accel_linear=0.5,
            accel_angular=6.0,
            use_dynamic_layer=False
        )
        
        self.logger.info(f"[SM] Coming home to ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
        self.executor.move_to(x, y, theta_deg, motion)
        # Note: In real system, this will trigger notify_action_completed when done
    
    def _on_enter_wait_to_come_home(self) -> None:
        """Handle entry into wait to come home state."""
        x, y, theta_deg = self.config.wait_to_come_home_pose
        motion = MotionParams(
            speed=1.0,
            end_speed=0.0,
            accel_linear=0.5,
            accel_angular=6.0,
            use_dynamic_layer=True
        )
        
        self.logger.info(f"[SM] Moving to wait position ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
        self.executor.move_to(x, y, theta_deg, motion)
        self.executor.execute_actuator_action('RESET_ACTUATORS')
    
    def _on_enter_end_of_match(self) -> None:
        """Handle end of match."""
        self.logger.info(f"[SM] Match ended! Final score: {self.match.get_score()}")
        self.executor.cancel_current_action()
    
    def _handle_idle_state(self) -> None:
        """Handle updates while in idle state."""
        # Check if we should return home
        if self.match.should_return_home(estimated_time_to_home=5.0):
            self.logger.info("[SM] Time to return home!")
            self.request_come_home()
    
    def _find_next_action(self) -> None:
        """Find and execute the next action from strategy."""
        if not self.strategy:
            self.logger.info("[SM] Strategy complete - no more actions")
            if self.on_strategy_completed:
                self.on_strategy_completed()
            return
        
        # Get next action
        action = self.strategy[0]
        
        # Check if action's tag was canceled
        if action.group and action.group in self.canceled_tags:
            self.logger.info(f"[SM] Skipping action with canceled tag '{action.group}'")
            self.strategy.pop(0)
            self._find_next_action()  # Try next action
            return
        
        # Execute action
        self.current_action = action
        self.current_tag = action.group
        self.strategy.pop(0)

        self.logger.info(f"[SM] Will now execute action: {action.action}" + (f" (group: {action.group})" if action.group else ""))
        
        self._execute_action(action)
    
    def _execute_action(self, action: Action) -> None:
        """Execute a single action."""
        self._transition_to(self.STATE_EXECUTING_ACTION)
        
        action_name = action.action
        
        try:
            if action_name == 'move':
                self._execute_move(action)
                
            elif action_name == 'detectPlatform':
                self._execute_detect_platform(action)
                
            elif action_name == 'wait':
                self._execute_wait(action)
                
            elif action_name == 'add_points':
                self._execute_add_points(action)
                
            elif action_name in self._get_actuator_actions():
                self._execute_actuator_action(action)
                
            else:
                self.logger.info(f"[SM] ERROR: Unknown action '{action_name}'")
                self.notify_action_completed()
                
        except Exception as e:
            self.logger.info(f"[SM] ERROR executing action: {e}")
            self.notify_action_completed()
    
    def _execute_move(self, action: Action) -> None:
        """Execute a move action."""
        if not action.target:
            self.logger.info("[SM] ERROR: Move action without target")
            self.notify_action_completed()
            return
        
        x, y, theta_deg = action.target.x, action.target.y, action.target.theta_deg
        
        # Apply offset if present
        if action.offset:
            theta_rad = theta_deg * math.pi / 180.0
            x += action.offset.x * math.cos(theta_rad) - action.offset.y * math.sin(theta_rad)
            y += action.offset.x * math.sin(theta_rad) + action.offset.y * math.cos(theta_rad)
            theta_deg += action.offset.theta_deg
        
        self.logger.info(f"[SM]   Move to ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
        self.executor.move_to(x, y, theta_deg, action.motion)
    
    def _execute_detect_platform(self, action: Action) -> None:
        """Execute platform detection."""
        self.logger.info("[SM]   Detecting platform...")
        self.executor.detect_platform()
        # Note: Real implementation will call notify_platform_detected when done
    
    def _execute_wait(self, action: Action) -> None:
        """Execute wait action."""
        duration = action.extra_params.get('duration', 1.0)
        self.logger.info(f"[SM]   Waiting {duration}s...")
        self.executor.wait(duration)
        # Note: Executor should call notify_action_completed when done
    
    def _execute_add_points(self, action: Action) -> None:
        """Execute add points action."""
        if action.points:
            reason = action.reason or "Points added"
            self.logger.info(f"[SM]   Added {action.points} points: \"{reason}\"")
            self.match.add_points(action.points, reason)
        
        # Points are added immediately, no waiting
        self.notify_action_completed()
    
    def _execute_actuator_action(self, action: Action) -> None:
        """Execute actuator action."""
        if self.config.simulation_mode:
            self.logger.info(f"[SM]   Actuator '{action.action}' skipped (simulation mode)")
            self.notify_action_completed()
        else:
            self.logger.info(f"[SM]   Executing actuator: {action.action}")
            self.executor.execute_actuator_action(action.action)
    
    def _cancel_current_action(self) -> None:
        """Cancel the current action."""
        if self.state == self.STATE_EXECUTING_ACTION:
            self.logger.info("[SM] Canceling current action")
            self.executor.cancel_current_action()
        
        self.current_action = None
        self._action_completed = True
    
    def _get_actuator_actions(self) -> list[str]:
        """Get list of valid actuator action names."""
        return [
            'PUT_BANNER',
            'TAKE_LOWER_PLANK', 'TAKE_UPPER_PLANK',
            'PUT_LOWER_PLANK_LAYER_1', 'PUT_UPPER_PLANK_LAYER_2',
            'TAKE_CANS_RIGHT', 'TAKE_CANS_LEFT',
            'PUT_CANS_LEFT_LAYER_1', 'PUT_CANS_RIGHT_LAYER_2',
            'RESET_ACTUATORS', 'GET_READY'
        ]
    
    # =================================================================
    # UTILITY METHODS
    # =================================================================
    
    def get_state(self) -> str:
        """Get current state."""
        return self.state
    
    def get_remaining_actions_count(self) -> int:
        """Get number of remaining actions in strategy."""
        return len(self.strategy)
    
    def is_executing(self) -> bool:
        """Check if currently executing an action."""
        return self.state == self.STATE_EXECUTING_ACTION
    
    def reset(self) -> None:
        """Reset the state machine."""
        self._cancel_current_action()
        self.strategy = []
        self.current_tag = None
        self.canceled_tags.clear()
        self.platform_center = None
        
        self._stop_requested = False
        self._action_completed = False
        self._platform_detected = False
        
        self._transition_to(self.STATE_STOP)
