#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Union
from enum import Enum
import math

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
        """Transforms the position for when we are on the blue team"""
        return Position(
            x=TABLE_WIDTH - self.x,
            y=self.y,
            theta_deg=(360 - self.theta_deg) % 360
        )

@dataclass
class Offset:
    """Represents a relative offset (not an absolute position)"""
    x: float
    y: float
    theta_deg: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        result = {"x": self.x, "y": self.y, "theta_deg": self.theta_deg, "is_offset": True}
        return result
    
    
# TODO ces params par défaut devront être lus dans le champi.config.yaml via ros
@dataclass
class MotionParams:
    """Motion parameters with default values"""
    speed: float = 1.0          # m/s
    end_speed: float = 0.0      # m/s
    accel_linear: float = 0.5   # m/s²
    accel_angular: float = 6.0  # rad/s²
    use_dynamic_layer: bool = False

@dataclass
class Action:
    """Base action with all possible parameters"""
    action: str
    target: Optional[Position] = None
    offset: Optional[Offset] = None
    group: Optional[str] = None  # Group name
    motion: MotionParams = field(default_factory=MotionParams)
    points: Optional[int] = None
    reason: Optional[str] = None
    # Other specific parameters
    extra_params: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        """Converts action to dictionary"""
        result = {"action": self.action}
        
        if self.target:
            result["target"] = self.target.to_dict()
        
        if self.offset:
            result["offset"] = self.offset.to_dict()
        
        if self.group:
            result["group"] = self.group
            
        if self.points is not None:
            result["points"] = self.points
            
        if self.reason:
            result["reason"] = self.reason
            
        # Add motion parameters only if they differ from defaults
        if self.motion.speed != 1.0:
            result["speed"] = self.motion.speed
        if self.motion.end_speed != 0.0:
            result["end_speed"] = self.motion.end_speed
        if self.motion.accel_linear != 0.5:
            result["accel_linear"] = self.motion.accel_linear
        if self.motion.accel_angular != 6.0:
            result["accel_angular"] = self.motion.accel_angular
        if self.motion.use_dynamic_layer:
            result["use_dynamic_layer"] = self.motion.use_dynamic_layer
            
        # Add extra parameters, converting Position/Offset objects to dict
        for key, value in self.extra_params.items():
            if isinstance(value, (Position, Offset)):
                result[key] = value.to_dict()
            else:
                result[key] = value
        
        return result

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
    
    def move_to(self, target: Union[Position, float], y: Optional[float] = None, theta_deg: float = 0.0, group: Optional[str] = None, **motion_kwargs) -> 'StrategyBuilder':
        """Add a movement action
        
        Can be called as:
        - move_to(Position(...), group=..., **motion_kwargs)
        - move_to(x, y, theta_deg, group=..., **motion_kwargs)
        """
        # Handle both Position object and (x, y, theta_deg) arguments
        if isinstance(target, Position):
            pos = target
        else:
            # target is x coordinate
            if y is None:
                raise ValueError("When providing x coordinate, y must also be provided")
            pos = Position(target, y, theta_deg)
        
        motion = MotionParams(**motion_kwargs)
        action = Action(
            action="move",
            target=pos,
            group=group or self.current_group,
            motion=motion
        )
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self

    def move_relative_to(self, target: Position, offset: Offset, group: Optional[str] = None, **motion_kwargs) -> 'StrategyBuilder':
        """Add a movement action with offset
        
        Args:
            target: Position object for the reference point
            offset: Offset object for the relative movement from target
            group: Optional group name
            **motion_kwargs: Motion parameters (speed, end_speed, accel_linear, accel_angular, use_dynamic_layer)
        """
        motion = MotionParams(**motion_kwargs)
        action = Action(
            action="move",
            target=target,
            offset=offset,
            group=group or self.current_group,
            motion=motion
        )
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def get_ready(self, group: Optional[str] = None) -> 'StrategyBuilder':
        """Add a GET_READY action"""
        action = Action(action="GET_READY", group=group or self.current_group)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def add_points(self, points: int, reason: str, group: Optional[str] = None) -> 'StrategyBuilder':
        """Add points"""
        action = Action(action="add_points", points=points, reason=reason, group=group or self.current_group)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def wait(self, duration: float, group: Optional[str] = None) -> 'StrategyBuilder':
        """Add a wait action"""
        action = Action(action="wait", group=group or self.current_group, extra_params={"duration": duration})
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    def custom_action(self, action_name: str, group: Optional[str] = None, **kwargs) -> 'StrategyBuilder':
        """Add a custom action"""
        action = Action(action=action_name, group=group or self.current_group, extra_params=kwargs)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    # Reusable functions for common sub-actions
    def put_banner(self, group: str = "banner") -> 'StrategyBuilder':
        """Complete sequence for placing the banner
        
        Args:
            target_position: Position object where to place the banner
            group: Group name for these actions
        """
        self.set_current_group(group)
        self.custom_action("PUT_BANNER")
        points = self.points_per_action["PUT_BANNER"]
        self.add_points(points, f"put_banner finished. {points} points for putting the banner", group=group)
        return self

    
    def take_elements_sequence(self, platform_center: Position, group: str = "elements") -> 'StrategyBuilder':
        """Complete sequence for taking elements
        
        Args:
            platform_center: Position object for the center of the platform
            group: Group name for these actions
            
        """
        self.set_current_group(group)

        # Approach movement
        self.move_relative_to(platform_center, Offset(-0.35, 0.0, 0.0))
        
        # Platform detection - offset from platform center
        # self.custom_action("detectPlatform")
        
        # Taking sequence - offsets from detected platform center
        self.move_relative_to(platform_center, Offset(-0.215, 0.0, -60.0))
        self.custom_action("TAKE_LOWER_PLANK")
        
        # Take left cans - offsets from platform center
        self.move_relative_to(platform_center, Offset(-0.25, -0.1, -60.0))
        self.move_relative_to(platform_center, Offset(-0.205, -0.1, -60.0))
        self.custom_action("TAKE_CANS_LEFT")
        
        # Take right cans - offsets from platform center
        self.move_relative_to(platform_center, Offset(-0.25, 0.1, 60.0))
        self.move_relative_to(platform_center, Offset(-0.205, 0.1, 60.0))
        self.custom_action("TAKE_CANS_RIGHT")
        
        return self
    
    def put_elements_sequence(self, target_position: Position, group: str = "elements") -> 'StrategyBuilder':
        """Complete sequence for placing elements
        
        Args:
            target_position: Position object where to place elements (target)
            group: Group name for these actions            
        """
        self.set_current_group(group)
        
        # First positioning - offset from target center
        self.move_relative_to(target_position, Offset(-0.21, 0.0, -60.0))

        self.custom_action("PUT_CANS_LEFT_LAYER_1")
        self.custom_action("PUT_LOWER_PLANK_LAYER_1")
        self.custom_action("TAKE_UPPER_PLANK")
        
        # Turn with RIGHT side facing - offset from target center
        self.move_relative_to(target_position, Offset(-0.21, 0.0, 60.0))
        
        self.custom_action("PUT_CANS_RIGHT_LAYER_2")
        
        # Turn with LEFT side facing - offset from target center
        self.move_relative_to(target_position, Offset(-0.21, 0.0, -60.0))
        
        self.custom_action("PUT_UPPER_PLANK_LAYER_2")
        
        # Add points
        points = self.points_per_action["2_LAYERS_STRUCTURE"]
        self.add_points(points, f"put_elements finished. {points} points for 2 layers structure", group=group)

        return self
    
    def come_home(self) -> 'StrategyBuilder':
        """Return home
        """
        if self.home_pose:
            self.move_to(self.home_pose, group="come_home")
        points = self.points_per_action["COME_HOME"]
        self.add_points(points, f"come_home finished. {points} points for coming home", group="come_home")
        return self
    
    def get_actions_by_group(self, group_name: str) -> List[Action]:
        """Get all actions belonging to a specific group"""
        return [action for action in self.actions if action.group == group_name]
    
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
                transformed_target = action.target.transform_for_blue() if action.target else None
                
                new_action = Action(
                    action=action.action,
                    target=transformed_target,
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
            if action.target:
                rotated_target = Position(action.target.x, action.target.y, action.target.theta_deg)
            
            action_copy = Action(
                action=action.action,
                target=rotated_target,
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