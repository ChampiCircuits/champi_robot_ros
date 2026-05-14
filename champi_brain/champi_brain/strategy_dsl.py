#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Union
from enum import Enum
import math

from champi_brain.actuator_commands import ActuatorCommand

"""
Default team color is YELLOW.
When converting for BLUE team, positions and angles are transformed accordingly.
The coordinate system assumes (0,0) at bottom-left
"""

TABLE_WIDTH = 3.0  # meters
TABLE_HEIGHT = 2.0  # meters

class Color(Enum):
    YELLOW = "YELLOW"
    BLUE = "BLUE"

class ActionGroup:
    """Represents a group of related actions"""
    def __init__(self, name: str):
        self.name = name
        self.actions: List['Action'] = []

@dataclass
class Position:
    x: float
    y: float
    theta_deg: float = 0.0
    
    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "theta_deg": self.theta_deg}
    
    def transform_for_blue(self) -> 'Position':
        """Transforms the position for when we are on the blue team
        
        Applies vertical axis symmetry (left-right inversion):
        - X coordinate is mirrored: x_blue = TABLE_WIDTH - x_yellow
        - Y coordinate stays the same
        - Angle is inverted horizontally: theta_blue = (180 - theta_yellow) % 360
          This keeps up/down directions but inverts left/right
        """
        return Position(
            x=TABLE_WIDTH - self.x,
            y=self.y,
            theta_deg=(180 - self.theta_deg) % 360
        )

@dataclass
class Offset:
    """Represents a relative offset from a target position.

    By default (world_frame=False) x/y are expressed in the **target's local frame**
    (rotated by target.theta) — useful when you want to approach "behind" or "in
    front of" an element regardless of where it sits on the table.

    Set world_frame=True when x/y are **absolute world-frame** displacements (not rotated).

    theta_deg behaviour:
    - theta_world_frame=False (default): theta_deg is *relative* to the target's
      orientation, so the approach angle auto-flips correctly when the target is
      mirrored for the blue team.
    - theta_world_frame=True: theta_deg is an **absolute world-frame angle**,
      ignoring the target's orientation entirely. Use this for asymmetric actuators
      that must always face the same direction regardless of team color.
      Example: a servo arm mounted on the left side of the robot always needs
      theta=180° (facing left) for both yellow and blue teams.
    """
    x: float
    y: float
    theta_deg: float = 0.0
    world_frame: bool = False       # If True, x/y are NOT rotated by target.theta
    theta_world_frame: bool = False # If True, theta_deg is absolute (NOT added to target theta)

    
@dataclass
class MotionParams:
    """Motion parameters with default values
    
    Default values MUST be configured using set_defaults() class method at startup.
    These defaults are read from ROS parameters.
    """
    # Class variables for defaults (configured at startup from ROS params)
    _default_speed: Optional[float] = None
    _default_end_speed: Optional[float] = None
    _default_accel_linear: Optional[float] = None
    _default_accel_angular: Optional[float] = None
    _default_use_collision_avoidance: Optional[bool] = None
    _default_linear_tolerance: Optional[float] = None
    _default_angular_tolerance: Optional[float] = None
    _default_max_angular_speed: Optional[float] = None

    # Instance variables - will be set in __post_init__
    speed: Optional[float] = None          # m/s
    end_speed: Optional[float] = None      # m/s
    accel_linear: Optional[float] = None   # m/s²
    accel_angular: Optional[float] = None  # rad/s²
    use_collision_avoidance: Optional[bool] = None # without collision avoidance, we do straight lines
    linear_tolerance: Optional[float] = None   # m
    angular_tolerance: Optional[float] = None  # rad
    max_angular_speed: Optional[float] = None  # rad/s

    def __post_init__(self):
        """Initialize instance variables with class defaults if not provided
        
        Raises RuntimeError if defaults haven't been configured via set_defaults()
        """
        if MotionParams._default_speed is None:
            raise RuntimeError(
                "MotionParams.set_defaults() must be called before creating instances. "
                "This should be done at node startup with values from ROS parameters."
            )
        
        if self.speed is None:
            self.speed = MotionParams._default_speed
        if self.end_speed is None:
            self.end_speed = MotionParams._default_end_speed
        if self.accel_linear is None:
            self.accel_linear = MotionParams._default_accel_linear
        if self.accel_angular is None:
            self.accel_angular = MotionParams._default_accel_angular
        if self.use_collision_avoidance is None:
            self.use_collision_avoidance = MotionParams._default_use_collision_avoidance
        if self.linear_tolerance is None:
            self.linear_tolerance = MotionParams._default_linear_tolerance
        if self.angular_tolerance is None:
            self.angular_tolerance = MotionParams._default_angular_tolerance
        if self.max_angular_speed is None:
            self.max_angular_speed = MotionParams._default_max_angular_speed

    @classmethod
    def set_defaults(cls, 
                     speed: float,
                     end_speed: float,
                     accel_linear: float,
                     accel_angular: float,
                     use_collision_avoidance: bool,
                     linear_tolerance: float,
                     angular_tolerance: float,
                     max_angular_speed: float) -> None:
        """Configure default values for all MotionParams instances
        
        This MUST be called once at startup with values from ROS parameters.
        """
        cls._default_speed = speed
        cls._default_end_speed = end_speed
        cls._default_accel_linear = accel_linear
        cls._default_accel_angular = accel_angular
        cls._default_use_collision_avoidance = use_collision_avoidance
        cls._default_linear_tolerance = linear_tolerance
        cls._default_angular_tolerance = angular_tolerance
        cls._default_max_angular_speed = max_angular_speed

@dataclass
class Action:
    """Base action with all possible parameters"""
    action: ActuatorCommand
    named_target: Optional[str] = None # Target by name in world state instead of target position
    pos_target: Optional[Position] = None
    offset: Optional[Offset] = None
    group: Optional[str] = None  # Group name
    motion: MotionParams = field(default_factory=MotionParams)
    points: Optional[int] = None
    reason: Optional[str] = None
    time: Optional[float] = None  # Time in seconds for the action
    # Other specific parameters
    extra_params: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate that either named_target or target is set, but not both"""
        if self.named_target is not None and self.pos_target is not None:
            raise ValueError(
                f"Action '{self.action}': cannot specify both 'named_target' and 'target'. "
                "Use either named_target (for world state references) or target (for absolute positions)."
            )
        # if self.named_target is None and self.pos_target is None:
        #     raise ValueError(
        #         f"Action '{self.action}': must specify either 'named_target' or 'target'. "
        #         "Use named_target for world state references or target for absolute positions."
        #     )


class StrategyBuilder:
    """Strategy builder with fluent DSL"""
    
    def __init__(self, points_per_action: Optional[dict] = None):
        self.actions: List[Action] = []
        self.groups: Dict[str, ActionGroup] = {}
        self.init_pose: Optional[Position] = None
        self.home_pose: Optional[Position] = None
        self.wait_to_come_home_pose: Optional[Position] = None
        self.current_group: Optional[str] = None
        self.points_per_action: dict = points_per_action or {}
        
    def create_group(self, name: str) -> 'StrategyBuilder':
        """Create a new action group"""
        self.groups[name] = ActionGroup(name)
        return self
    
    def set_current_group(self, group_name: str) -> 'StrategyBuilder':
        """Set the current group for subsequent actions"""
        if group_name not in self.groups:
            self.create_group(group_name)
        self.current_group = group_name
        return self
        
    def set_init_pose(self, x: float, y: float, theta_deg: float) -> 'StrategyBuilder':
        """Set the initial pose"""
        self.init_pose = Position(x, y, theta_deg)
        return self
    
    def set_home_pose(self, x: float, y: float, theta_deg: float) -> 'StrategyBuilder':
        """Set the home pose"""
        self.home_pose = Position(x, y, theta_deg)
        return self
    
    def set_wait_to_come_home_pose(self, x: float, y: float, theta_deg: float) -> 'StrategyBuilder':
        """Set the wait-to-come-home pose"""
        self.wait_to_come_home_pose = Position(x, y, theta_deg)
        return self
    
    def move_relative_to(self, target: Union[Position, str], offset: Offset, group: Optional[str] = None, **motion_kwargs) -> 'StrategyBuilder':
        """Add a movement action with offset
        
        Args:
            target: Position object or string (named target in world state) for the reference point
            offset: Offset object for the relative movement from target
            group: Optional group name
            **motion_kwargs: Motion parameters (speed, end_speed, accel_linear, accel_angular, use_collision_avoidance)
        """
        motion = MotionParams(**motion_kwargs)
        
        # Determine if target is a Position or a named target string
        if isinstance(target, str):
            named_target = target
            pos_target = None
        else:
            pos_target = target
            named_target = None

        action = Action(
            action=ActuatorCommand.MOVE,
            pos_target=pos_target,
            named_target=named_target,
            offset=offset,
            group=group or self.current_group,
            motion=motion
        )
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def move_to(self, target: Union[Position, str], group: Optional[str] = None, **motion_kwargs) -> 'StrategyBuilder':
        """Add a movement action

        Args:
            target: Position object or string (named target in world state) for the reference point
            group: Optional group name
            **motion_kwargs: Motion parameters (speed, end_speed, accel_linear, accel_angular, use_collision_avoidance)
        """
        self.move_relative_to(target, Offset(0.0, 0.0, 0.0), group=group, **motion_kwargs)
        return self
    
    def get_ready(self, group: Optional[str] = None) -> 'StrategyBuilder':
        """Add a GET_READY action"""
        action = Action(action=ActuatorCommand.GET_READY, group=group or self.current_group)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def add_points(self, points: int, reason: str, group: Optional[str] = None) -> 'StrategyBuilder':
        """Add points"""
        action = Action(action=ActuatorCommand.ADD_POINTS, points=points, reason=reason, group=group or self.current_group)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def wait(self, duration: float, group: Optional[str] = None) -> 'StrategyBuilder':
        """Add a wait action"""
        action = Action(action=ActuatorCommand.WAIT, group=group or self.current_group, extra_params={"duration": duration})
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def custom_action(self, action_name: ActuatorCommand, group: Optional[str] = None, **kwargs) -> 'StrategyBuilder':
        """Add a custom action"""
        action = Action(action=action_name, group=group or self.current_group, extra_params=kwargs)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    # Reusable functions for common sub-actions
    def move_thermometer(self, group: str) -> 'StrategyBuilder':
        """Complete sequence for placing the banner
        
        Args:
            group: Group name for these actions
        """
        self.set_current_group(group)

        # thermometer initial position is on the rightmost of its slider.
        # we have to move it in the center of the slider
        # thermometer_initial_position = Position(1.2, 0.0, 0.0) # for Yellow team
        thermometer_initial_position = Position(1.4, 0.0, 0.0) # for Yellow team # TODO ca c'est la bonne pose, mais la table est trop petite là
        thermometer_target_position = Position(0.68, 0.0, 0.0)  # for Yellow team

        # world_frame=True  → x/y offsets are absolute (not rotated by target theta)
        # theta_world_frame=True → robot always faces 180° (left) regardless of team color,
        #                          because the servo arm is physically on one fixed side of the robot
        self.move_relative_to(thermometer_initial_position, Offset(0.05, 0.17, 30.0, world_frame=True, theta_world_frame=True), use_collision_avoidance=True)
        self.custom_action(ActuatorCommand.THERMOMETER_LOWER_SERVO)
        self.move_relative_to(thermometer_target_position, Offset(0.05, 0.15, 30.0, world_frame=True, theta_world_frame=True), linear_tolerance=0.0025, speed=0.1)
        self.custom_action(ActuatorCommand.THERMOMETER_RAISE_SERVO)

        points = self.points_per_action["THERMOMETER"]
        self.add_points(points, f"move_thermometer finished. {points} points for moving the thermometer", group=group)
        return self

    
    def take_elements_sequence(self, elements_center: Union[Position, str], which_actuator: str, group: str) -> 'StrategyBuilder':
        """Complete sequence for taking elements
        
        Args:
            elements_center: Union[Position, str] object for the center of the platform or named target
            which_actuator: string to specify which actuator to use for taking ("LEFT" OR "RIGHT")
            group: Group name for these actions
            
        """
        self.set_current_group(group)

        if which_actuator == 'LEFT':
            offset_angle = -60.0
            lower_actuator_command = ActuatorCommand.LOWER_LEFT_ARM
            get_ready_actuator_command = ActuatorCommand.GET_READY_LEFT_ARM
        elif which_actuator == 'RIGHT':
            offset_angle = +60.0
            lower_actuator_command = ActuatorCommand.LOWER_RIGHT_ARM
            get_ready_actuator_command = ActuatorCommand.GET_READY_RIGHT_ARM
        else:
            raise ValueError(f"Invalid actuator specified: {which_actuator}. Must be 'LEFT' or 'RIGHT'.")

        # Approach movement
        self.move_relative_to(elements_center, Offset(-0.35, 0.0, 0.0), use_collision_avoidance=True)
        self.wait(1.0)  # wait a bit to stabilize before detection

        # NutBoxes detection — after this action, "<group>" is available in world state as the position of detected nut boxes
        self.custom_action(ActuatorCommand.DETECT_NUTBOXES)
        self.custom_action(ActuatorCommand.STORE_PENDING_MASK)
        # Move relative to the detected position rather than the theoretical center
        # We use the group name as the label of the detected element. Same when inserting into world state
        self.move_relative_to(group, Offset(-0.35, 0.0, offset_angle), linear_tolerance=0.001, angular_tolerance=0.05)

        self.custom_action(get_ready_actuator_command, element_taken=group)  # it will also raise it directly after

        self.move_relative_to(group, Offset(-0.225, 0.0, offset_angle), linear_tolerance=0.001, angular_tolerance=0.05)
        
        # Taking boxes — embed element_taken so the planner removes the element obstacle
        self.custom_action(lower_actuator_command, element_taken=group)  # it will also raise it directly after

        return self
    
    def put_elements_sequence(self, target_position: Union[Position, str], which_actuator: str, group: str,
                               zone_id: str = None) -> 'StrategyBuilder':
        """Complete sequence for placing elements
        
        Args:
            target_position: Union[Position, str] object where to place elements or named target
            which_actuator: string to specify which actuator to use for placing ("LEFT" OR "RIGHT")
            group: Group name for these actions
            zone_id: Zone id to mark as occupied after placing (e.g. 'garde_manger_2')
        """
        self.set_current_group(group)

        if which_actuator == 'LEFT':
            offset_angle = -60.0
            actuator_command = ActuatorCommand.LET_GO_ELEMENTS_LEFT_ARM
        elif which_actuator == 'RIGHT':
            offset_angle = +60.0
            actuator_command = ActuatorCommand.LET_GO_ELEMENTS_RIGHT_ARM
        else:
            raise ValueError(f"Invalid actuator specified: {which_actuator}. Must be 'LEFT' or 'RIGHT'.")
        
        self.move_relative_to(target_position, Offset(-0.3, 0.0, offset_angle), use_collision_avoidance=False, linear_tolerance=0.001, angular_tolerance=0.05)
        # Embed zone_occupied so the planner marks the target zone as occupied after placing
        self.custom_action(actuator_command, zone_occupied=zone_id)
        self.move_relative_to(target_position, Offset(-0.4, 0.0, offset_angle), use_collision_avoidance=False, linear_tolerance=0.001, angular_tolerance=0.05)

        # Add points
        points = self.points_per_action["PUT_4_BOXES_OUT_PLUS_BONUS"]
        self.add_points(points, f"PUT_4_BOXES_OUT_PLUS_BONUS finished. {points} points", group=group)

        return self
    
    def come_home(self) -> 'StrategyBuilder':
        """Return home
        """
        # if self.home_pose:
            # self.move_relative_to(self.home_pose, Offset(0, 0.4, 0.0), group="come_home") # to avoid the grenier
            # self.move_to(self.home_pose, group="come_home")
        points = self.points_per_action["COME_HOME"]
        self.add_points(points, f"come_home finished. {points} points for coming home", group="come_home")
        self.custom_action(ActuatorCommand.PUMPS_OFF)
        return self
    
    def get_actions_by_group(self, group: str) -> List[Action]:
        """Get all actions belonging to a specific group"""
        return [action for action in self.actions if action.group == group]
    
    def get_group_names(self) -> List[str]:
        """Get all group names"""
        return list(self.groups.keys())
    
    def get_transformed_actions(self, color: Color) -> List[Action]:
        """Get list of transformed Action objects (keeps typing) for the given color"""
        actions = self.actions
        
        # Transform for blue team
        if color == Color.BLUE:
            actions = []
            for action in self.actions:
                transformed_target = action.pos_target.transform_for_blue() if action.pos_target else None

                new_action = Action(
                    action=action.action,
                    named_target=action.named_target,
                    pos_target=transformed_target,
                    offset=action.offset,  # Offsets stay relative, not transformed
                    group=action.group,
                    motion=action.motion,
                    points=action.points,
                    reason=action.reason,
                    extra_params=action.extra_params.copy()
                )
                actions.append(new_action)
        
        actions_rotated = []
        for action in actions:
            rotated_target = None
            if action.pos_target:
                rotated_target = Position(action.pos_target.x, action.pos_target.y, action.pos_target.theta_deg)
            
            action_copy = Action(
                action=action.action,
                named_target=action.named_target,
                pos_target=rotated_target,
                offset=action.offset,  # Offsets stay as-is
                group=action.group,
                motion=action.motion,
                points=action.points,
                reason=action.reason,
                extra_params=action.extra_params.copy()
            )
            actions_rotated.append(action_copy)
        
        return actions_rotated
    
    def get_init_pose(self, color: Color) -> Position:
        """Get transformed init pose"""
        if not self.init_pose:
            raise ValueError("Init pose must be set")
        
        init_pose = self.init_pose
        if color == Color.BLUE:
            init_pose = init_pose.transform_for_blue()
        
        return Position(init_pose.x, init_pose.y, init_pose.theta_deg)
    
    def get_home_pose(self, color: Color) -> Position:
        """Get transformed home pose"""
        if not self.home_pose:
            raise ValueError("Home pose must be set")
        
        home_pose = self.home_pose
        if color == Color.BLUE:
            home_pose = home_pose.transform_for_blue()
        
        return Position(home_pose.x, home_pose.y, home_pose.theta_deg)
    
    def get_wait_to_come_home_pose(self, color: Color) -> Position:
        """Get transformed wait-to-come-home pose"""
        if not self.wait_to_come_home_pose:
            raise ValueError("Wait-to-come-home pose must be set")
        
        wait_pose = self.wait_to_come_home_pose
        if color == Color.BLUE:
            wait_pose = wait_pose.transform_for_blue()
        
        return Position(wait_pose.x, wait_pose.y, wait_pose.theta_deg)