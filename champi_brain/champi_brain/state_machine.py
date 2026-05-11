#!/usr/bin/env python3

from math import degrees, radians, sin, cos
from typing import List, Optional, Callable, Dict, Tuple
from dataclasses import dataclass
from rclpy.logging import get_logger
import rclpy

from champi_brain.action_executor.action_executor import ActionExecutor
from champi_brain.match_controller import MatchController
from champi_brain.strategy_dsl import Action, MotionParams, Offset
from champi_brain.world_state.symmetry import get_element_id_for_color
from champi_brain.enums import Color
from champi_brain.actuator_commands import ActuatorCommand
from champi_interfaces.msg import GameElement
from champi_libraries_py.utils.angles import get_yaw


@dataclass
class StrategyConfig:
    """Configuration for the state machine."""
    color: str  # 'blue' or 'yellow'
    init_pose: tuple[float, float, float]  # (x, y, theta_deg)
    home_pose: tuple[float, float, float]  # (x, y, theta_deg)
    wait_to_come_home_pose: tuple[float, float, float]  # (x, y, theta_deg)
    come_home_points: int = 0  # Points awarded for returning home
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
        match_controller: MatchController
    ):
        """
        Initialize the state machine.
        
        Args:
            executor: ActionExecutor implementation for robot actions
            match_controller: MatchController for timing and scoring
        """
        self.executor = executor
        self.match = match_controller
        self.logger = get_logger('SM')
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Configuration (set later via set_config)
        self.strategy_config: Optional[StrategyConfig] = None
        
        # State
        self.state = self.STATE_STOP
        self.strategy: List[Action] = []
        self.current_action: Optional[Action] = None
        self.last_dispatched_action: Optional[Action] = None
        self.current_group: Optional[str] = None
        self.canceled_groups: set[str] = set()
        
        # World state (updated by ROS node)
        self.world_state_elements: Dict[str, GameElement] = {}
        
        # Flags for state transitions
        self._stop_requested = False
        self._action_completed = False
        self._nutboxes_detected = False
        self.nutboxes_center_pose_in_world: Optional[tuple[float, float, float]] = None

        # Initialization flags
        self._ros_initialized = False
        self._config_chosen = False
        self._tirette_released = False
        
        # Callbacks for external events
        self.on_state_changed: Optional[Callable[[str], None]] = None
        self.on_strategy_completed: Optional[Callable[[], None]] = None

    # =================================================================
    # INITIALIZATION METHODS
    # =================================================================
    
    def start_initialization(self) -> None:
        """Start the initialization sequence."""
        self._transition_to(self.STATE_INIT)
    
    def set_strategy(self, strategy: List[Action], config: StrategyConfig) -> None:
        """Set the strategy (list of actions) to execute."""
        self.strategy = strategy.copy()
        self.strategy_config = config
        self._check_init_progress()

    def update_world_state(self, elements: Dict[str, GameElement]) -> None:
        """Update the world state with new element positions.
        
        Args:
            elements: Dict of {element_id: GameElement (ROS message)}
        """
        self.world_state_elements = elements
        self.logger.debug(f"World state updated: {len(elements)} elements")
        self._check_init_progress()
        
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

    # =================================================================
    # PUBLIC API METHODS
    # =================================================================

    def request_stop(self) -> None:
        """Request emergency stop."""
        if self.state == self.STATE_END_OF_MATCH:
            return  # Match already over, ignore stop request
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
        elif self.state == self.STATE_COME_HOME:
            if self.strategy_config and self.strategy_config.come_home_points > 0:
                self.logger.info(f"Added {self.strategy_config.come_home_points} points for coming home")
                self.match.add_points(self.strategy_config.come_home_points)

    def notify_nutboxes_detected(self, nutboxes_center_pose_in_world: tuple[float, float, float]) -> None:
        """Notify that nutboxes have been detected. Injects the position into
        world_state_elements under the key 'detected_nutboxes' so that subsequent
        move_relative_to('detected_nutboxes', ...) actions resolve at runtime."""
        x, y, theta_deg = nutboxes_center_pose_in_world
        self.nutboxes_center_pose_in_world = nutboxes_center_pose_in_world
        self._nutboxes_detected = True

        elem = GameElement()
        elem.id = self.current_group if self.current_group else "detected_nutboxes"
        elem.pose.position.x = x
        elem.pose.position.y = y
        t = radians(theta_deg)
        elem.pose.orientation.z = sin(t / 2)
        elem.pose.orientation.w = cos(t / 2)
        self.world_state_elements[elem.id] = elem
        self.logger.info(f"📦 Nutboxes ({elem.id}) injected into world state at ({x:.3f}, {y:.3f}, {theta_deg:.1f}°)")

        self.notify_action_completed()
        
    def cancel_current_group(self) -> None:
        """Cancel all actions with the current group."""
        if self.current_group:
            self.canceled_groups.add(self.current_group)
        self.world_state_elements.pop("detected_nutboxes", None)
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
            return  # Always return when match is over - END_OF_MATCH is terminal
        
        # Check if we should return home (in any state except already coming home or ended)
        if self.match.is_match_started():
            if self.state not in [self.STATE_COME_HOME, self.STATE_END_OF_MATCH, self.STATE_STOP]:
                # TODO: Ask path planner for real estimated time to home based on current position
                estimated_time_to_home = 5.0
                if self.match.should_return_home(estimated_time_to_home):
                    self.logger.warn(f"Time to return home! (remaining: {self.match.get_remaining_time():.1f}s)")
                    self.request_come_home()
                    return
        

    
    # =================================================================
    # INTERNAL STATE MACHINE LOGIC
    # =================================================================
    
    def _transition_to(self, new_state: str) -> None:
        """Transition to a new state."""
        if self.state == new_state:
            return
            
        old_state = self.state
        self.state = new_state
        
        self.logger.debug(f"Transition: {old_state} -> {new_state} \t time left: {self.match.get_remaining_time():.1f}s\n")
        
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
        # Check each initialization step in order
        if not self._ros_initialized:
            self.logger.info("Waiting for STRAT/ros initialization...")
            return
            
        if not self._config_chosen:
            self.logger.info("Waiting for user to choose configuration (or auto-placement)...", throttle_duration_sec=1.)
            return
                    
        if not self.strategy_config:
            self.logger.error("Strategy configuration not set!")
            return
        
        if not self.world_state_elements:
            self.logger.info("Waiting for first world state to be available...")
            return
            
        if not self._tirette_released:
            self.logger.info("Waiting for tirette release...")
            return
        
        # All initialization steps complete - start match
        self.logger.info("✅ Initialization complete! Starting match...")
        self.match.start_match()
        self._transition_to(self.STATE_IDLE)
    
    def _on_enter_idle(self) -> None:
        """Handle entry into idle state - find next action."""
        self.current_action = None
        self._action_completed = False
        self._nutboxes_detected = False
        
        # Process next action from strategy
        self._find_next_action()
    
    def _on_enter_come_home(self) -> None:
        """Handle entry into come home state."""
        if not self.strategy_config:
            raise ValueError("Cannot come home: no strategy configuration set!")

        x, y, theta_deg = self.strategy_config.home_pose
        motion = MotionParams(
            speed=1.0,
            end_speed=0.0,
            accel_linear=0.5,
            accel_angular=6.0,
            use_collision_avoidance=False
        )
        
        self.logger.info(f"Coming home to ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
        self.executor.move_to(x, y, theta_deg, motion)
        # TODO: sometimes logs are strange when canceling nav goals (cancel and then re asked again same goal...)
    
    def _on_enter_wait_to_come_home(self) -> None:
        """Handle entry into wait to come home state."""
        if not self.strategy_config:
            raise ValueError("Cannot go to wait position: no strategy configuration set!")

        x, y, theta_deg = self.strategy_config.wait_to_come_home_pose
        motion = MotionParams(
            speed=1.0,
            end_speed=0.0,
            accel_linear=0.5,
            accel_angular=6.0,
            use_collision_avoidance=True
        )
        
        self.logger.info(f"Moving to wait position ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
        self.executor.move_to(x, y, theta_deg, motion)
        self.executor.execute_actuator_action(ActuatorCommand.RESET_ACTUATOR)
    
    def _on_enter_end_of_match(self) -> None:
        """Handle end of match."""
        self.logger.info(f"Match ended! Final score: {self.match.get_score()}")
        self.executor.cancel_current_action()
    

    
    def _find_next_action(self) -> None:
        """Find and execute the next action from strategy."""
        if not self.strategy:
            self.logger.info("Strategy complete - no more actions")
            if self.on_strategy_completed:
                self.on_strategy_completed()
            return
        
        # Get next action
        action = self.strategy[0]
        
        # Check if action's group was canceled
        if action.group and action.group in self.canceled_groups:
            self.logger.info(f"Skipping action {action.action} with canceled group '{action.group}'")
            self.strategy.pop(0)
            self._find_next_action()  # Try next action
            return
        
        # Execute action
        self.current_action = action
        self.current_group = action.group
        self.last_dispatched_action = action
        self.strategy.pop(0)

        self.logger.info(f"Will now execute action: {action.action}" + (f" (group: {action.group})" if action.group else ""))

        self._execute_action(action)
    
    def _execute_action(self, action: Action) -> None:
        """Execute a single action."""
        self._transition_to(self.STATE_EXECUTING_ACTION)

        actuator_command: ActuatorCommand = action.action
    
        if actuator_command == ActuatorCommand.MOVE:
            self._execute_move(action)
        elif actuator_command == ActuatorCommand.DETECT_NUTBOXES:
            self._execute_detect_nutboxes(action)
        elif actuator_command == ActuatorCommand.WAIT:
            self._execute_wait(action)
        elif actuator_command == ActuatorCommand.ADD_POINTS:
            self._execute_add_points(action)

        elif actuator_command in ActuatorCommand.__members__.values():
            self._execute_actuator_action(action)
        else:
            raise ValueError(f"Unknown action={actuator_command}. Possible actions are: {list(ActuatorCommand.__members__.keys())}")

    
    def _execute_move(self, action: Action) -> None:
        """Execute a move action."""
        # Get target position
        target_position = self._get_target_position(action)
        if target_position is None:
            self.notify_action_completed()
            return
        
        x, y, theta_deg = target_position
        
        # Apply offset if present
        if action.offset:
            x, y, theta_deg = self._apply_offset(x, y, theta_deg, action.offset)

        self.logger.info(f"Move to ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
        self.executor.move_to(x, y, theta_deg, action.motion)
    
    def _get_target_position(self, action: Action) -> Optional[Tuple[float, float, float]]:
        """Get target position from action.
        
        Returns:
            Tuple of (x, y, theta_deg) or None if target not found
        """
        if action.pos_target is not None:
            # Absolute position specified
            return (action.pos_target.x, action.pos_target.y, action.pos_target.theta_deg)
            
        if action.named_target is not None:
            if not self.strategy_config:
                raise ValueError("Cannot resolve named target: no strategy configuration set!")
            
            # Get target from world state by name
            base_target_name = action.named_target
            # Apply symmetry mapping based on team color            
            team_color = Color.BLUE if self.strategy_config.color.upper() == 'BLUE' else Color.YELLOW
            target_name = get_element_id_for_color(base_target_name, team_color)
            
            if target_name not in self.world_state_elements:
                raise ValueError(f"Element '{target_name}' (from '{base_target_name}' for {team_color}) not found in world state!")
            
            # Get element position from world state (ROS GameElement message)
            element = self.world_state_elements[target_name]
            x = element.pose.position.x
            y = element.pose.position.y
            theta_deg = degrees(get_yaw(element.pose))
            self.logger.info(f"Target '{target_name}' found at ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
            return (x, y, theta_deg)

        raise ValueError("No target specified for move action!")
    
    def _apply_offset(self, x: float, y: float, theta_deg: float,
                      offset: Offset) -> Tuple[float, float, float]:
        """Apply an offset expressed in the target's local frame.

        Coordinate convention (target's local frame):
          +x  = forward  (direction the target faces)
          +y  = left     (perpendicular, CCW from +x)
          theta_deg = added to the target's own orientation
                      → 0   : robot faces the same direction as the target
                      → 180 : robot faces opposite to the target

        Examples
        --------
        Offset(-0.35, 0, 0)  → 35 cm behind the target, same heading
                                (robot front points toward the target)
        Offset( 0.10, 0, 0)  → 10 cm in front of the target, same heading
                                (robot back points toward the target — useful
                                 for rear-mounted actuators)
        Offset( 0.05, 0.1, 180) → 5 cm in front, 10 cm to the left,
                                   facing opposite to the target
        """
        theta_rad = radians(theta_deg)
        cos_t = cos(theta_rad)
        sin_t = sin(theta_rad)

        if offset.world_frame:
            # x/y are world-frame displacements — no rotation needed.
            new_x = x + offset.x
            new_y = y + offset.y
        else:
            # x/y are in the target's local frame — rotate by target theta.
            new_x = x + offset.x * cos_t - offset.y * sin_t
            new_y = y + offset.x * sin_t + offset.y * cos_t

        if offset.theta_world_frame:
            # theta_deg is an absolute world-frame angle — ignore target orientation.
            # Use this for asymmetric actuators that must always face the same
            # direction regardless of team color (e.g. a servo arm on a fixed side).
            new_theta = offset.theta_deg
        else:
            # theta_deg is relative to the target's orientation — auto-flips for blue.
            new_theta = theta_deg + offset.theta_deg
        return new_x, new_y, new_theta

    def _execute_detect_nutboxes(self, action: Action) -> None:
        """Execute nutbox detection."""
        self.logger.info("Detecting nutboxes...")
        self.executor.detect_nutboxes()
    
    def _execute_wait(self, action: Action) -> None:
        """Execute wait action."""
        duration = action.extra_params['duration']
        self.logger.info(f"Waiting {duration}s...")
        self.executor.wait(duration)
        # Note: Executor should call notify_action_completed when done # TODO?
    
    def _execute_add_points(self, action: Action) -> None:
        """Execute add points action."""
        if not action.points:
            self.logger.error(f"No points specified for add_points action!")
            self.notify_action_completed()
            return

        self.logger.info(f"Added {action.points} points: \"{action.reason}\"")
        self.match.add_points(action.points)
        self.notify_action_completed()
    
    def _execute_actuator_action(self, action: Action) -> None:
        """Execute actuator action."""
        self.logger.info(f"Executing actuator: {action.action}")
        self.executor.execute_actuator_action(action.action)
        # Propagate obstacle state changes embedded in this action's metadata
        if 'element_taken' in action.extra_params:
            self.executor.set_obstacle_state(action.extra_params['element_taken'], False)
        zone_id = action.extra_params.get('zone_occupied')
        if zone_id is not None:
            self.executor.set_obstacle_state(zone_id, True)
    
    def _cancel_current_action(self) -> None:
        """Cancel the current action."""
        if self.state == self.STATE_EXECUTING_ACTION:
            self.logger.warn("Cancelling current action")
            self.executor.cancel_current_action()
        
        self.current_action = None
        self._action_completed = True
    
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
        self.current_group = None
        self.canceled_groups.clear()
        self.nutboxes_center_pose_in_world = None
        self.world_state_elements.pop("detected_nutboxes", None)
        
        self._stop_requested = False
        self._action_completed = False
        self._nutboxes_detected = False

        # Reset init flags so the full init sequence is replayed after reset
        self._ros_initialized = False
        self._config_chosen = False
        self._tirette_released = False
        
        self._transition_to(self.STATE_STOP)
