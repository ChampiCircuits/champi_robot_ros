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
            
        # Add extra parameters, converting Position objects to dict
        for key, value in self.extra_params.items():
            if isinstance(value, Position):
                result[key] = value.to_dict()
            else:
                result[key] = value
        
        return result

class StrategyBuilder:
    """Strategy builder with fluent DSL"""
    
    def __init__(self):
        self.actions: List[Action] = []
        self.groups: Dict[str, ActionGroup] = {}
        self.init_pose: Optional[Position] = None
        self.home_pose: Optional[Position] = None
        self.current_group: Optional[str] = None
        
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
    
    def move_to(self, x: float, y: float, theta_deg: float = 0.0, 
                group: Optional[str] = None, **motion_kwargs) -> 'StrategyBuilder':
        """Add a movement action"""
        motion = MotionParams(**motion_kwargs)
        action = Action(
            action="move",
            target=Position(x, y, theta_deg),
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
    
    def custom_action(self, action_name: str, group: Optional[str] = None, **kwargs) -> 'StrategyBuilder':
        """Add a custom action"""
        action = Action(action=action_name, group=group or self.current_group, extra_params=kwargs)
        self.actions.append(action)
        if action.group and action.group in self.groups:
            self.groups[action.group].actions.append(action)
        return self
    
    # Reusable functions for common sub-actions
    def put_banner(self, x: float, y: float, theta_deg: float = 0.0, group: str = "banner") -> 'StrategyBuilder':
        """Complete sequence for placing the banner
        
        Args:
            x, y, theta_deg: Absolute position where to place the banner
            group: Group name for these actions
            
        Note: The original YAML had no move action (commented out), so we just do the action directly.
        """
        self.set_current_group(group)
        # Note: Original YAML had this move commented out:
        # move to { x: 0.0, y: 0.0, theta_deg: -60.0 } as offset
        self.custom_action("PUT_BANNER")
        self.add_points(20, "put_banner finished. 20 points for putting the banner")
        return self
    
    def _apply_offset(self, base_x: float, base_y: float, base_theta_deg: float,
                     offset_x: float, offset_y: float, offset_theta_deg: float) -> Position:
        """Apply rotation and translation to transform an offset into an absolute position"""
        theta_rad = math.radians(base_theta_deg)
        
        # Apply rotation and translation
        x_transformed = (offset_x * math.cos(theta_rad) - offset_y * math.sin(theta_rad)) + base_x
        y_transformed = (offset_x * math.sin(theta_rad) + offset_y * math.cos(theta_rad)) + base_y
        theta_transformed = (offset_theta_deg + base_theta_deg) % 360
        
        return Position(x_transformed, y_transformed, theta_transformed)
    
    def take_elements_sequence(self, x: float, y: float, theta_deg: float = 0.0, group: str = "elements") -> 'StrategyBuilder':
        """Complete sequence for taking elements
        
        Args:
            x, y, theta_deg: Absolute position where the center of the platform is located
            group: Group name for these actions
            
        Note: Offsets are relative to the platform center position and are transformed accordingly.
              'move' and 'detectPlatform' actions get transformed offsets (absolute positions).
              'moveForPlatform' actions keep their offsets as-is (robot transforms them after detection).
        """
        self.set_current_group(group)
        
        # Approach movement - transform offset (0, -0.35, 0) to absolute position
        approach_pos = self._apply_offset(x, y, theta_deg, 0.0, -0.35, 0.0)
        self.move_to(approach_pos.x, approach_pos.y, approach_pos.theta_deg)
        
        # Platform detection - use same transformed position as move (absolute, not offset!)
        self.custom_action("detectPlatform", target=approach_pos)
        
        # Taking sequence - these are offsets for moveForPlatform (not transformed, robot will handle)
        self.custom_action("moveForPlatform", target=Position(0.0, -0.215, -60.0))
        self.custom_action("TAKE_LOWER_PLANK")
        
        # Take left cans - offsets for moveForPlatform
        self.custom_action("moveForPlatform", target=Position(-0.1, -0.25, -60.0))
        self.custom_action("moveForPlatform", target=Position(-0.1, -0.205, -60.0))
        self.custom_action("TAKE_CANS_LEFT")
        
        # Take right cans - offsets for moveForPlatform
        self.custom_action("moveForPlatform", target=Position(0.1, -0.25, 60.0))
        self.custom_action("moveForPlatform", target=Position(0.1, -0.205, 60.0))
        self.custom_action("TAKE_CANS_RIGHT")
        
        return self
    
    def put_elements_sequence(self, x: float, y: float, theta_deg: float = 180.0, group: str = "elements") -> 'StrategyBuilder':
        """Complete sequence for placing elements
        
        Args:
            x, y, theta_deg: Absolute position where to place elements (center of the jardiniere)
            group: Group name for these actions
            
        Note: Uses offsets from put_elements.yaml:
              - move to offset (0, -0.21, -60°) for initial positioning
              - Various PUT actions
              - move to offset (0, -0.21, 60°) to turn with RIGHT side
              - move to offset (0, -0.21, -60°) to turn with LEFT side
        """
        self.set_current_group(group)
        
        # First positioning - transform offset (0, -0.21, -60°)
        pos1 = self._apply_offset(x, y, theta_deg, 0.0, -0.21, -60.0)
        self.move_to(pos1.x, pos1.y, pos1.theta_deg)
        
        self.custom_action("PUT_CANS_LEFT_LAYER_1")
        self.custom_action("PUT_LOWER_PLANK_LAYER_1")
        self.custom_action("TAKE_UPPER_PLANK")
        
        # Turn with RIGHT side facing - transform offset (0, -0.21, 60°)
        pos2 = self._apply_offset(x, y, theta_deg, 0.0, -0.21, 60.0)
        self.move_to(pos2.x, pos2.y, pos2.theta_deg)
        
        self.custom_action("PUT_CANS_RIGHT_LAYER_2")
        
        # Turn with LEFT side facing - transform offset (0, -0.21, -60°)
        pos3 = self._apply_offset(x, y, theta_deg, 0.0, -0.21, -60.0)
        self.move_to(pos3.x, pos3.y, pos3.theta_deg)
        
        self.custom_action("PUT_UPPER_PLANK_LAYER_2")
        
        # Add points
        self.add_points(12, "put_elements finished. 4 points per layer of level 1 + 8 points per layer of level 2")
        
        return self
    
    def come_home(self) -> 'StrategyBuilder':
        """Return home
        
        Note: Uses offset (0, 0, 0) from come_home.yaml, which means we just go to the exact home pose.
        """
        if self.home_pose:
            # Offset is (0, 0, 0), so just go directly to home_pose
            self.move_to(self.home_pose.x, self.home_pose.y, self.home_pose.theta_deg, group="come_home")
        self.add_points(10, "come_home finished. 10 points for coming home", group="come_home")
        return self
    
    def get_actions_by_group(self, group_name: str) -> List[Action]:
        """Get all actions belonging to a specific group"""
        return [action for action in self.actions if action.group == group_name]
    
    def get_group_names(self) -> List[str]:
        """Get all group names"""
        return list(self.groups.keys())
    
    def to_dict(self, color: Color = Color.YELLOW) -> Dict[str, Any]:
        """Convert strategy to dictionary"""
        if not self.init_pose or not self.home_pose:
            raise ValueError("Init pose and home pose must be set")
        
        init_pose = self.init_pose
        home_pose = self.home_pose
        actions = self.actions
        
        # Transform for blue team
        if color == Color.BLUE:
            init_pose = self.init_pose.transform_for_blue()
            home_pose = self.home_pose.transform_for_blue()
            # Transform actions too
            actions = []
            for action in self.actions:
                new_action = Action(
                    action=action.action,
                    target=action.target.transform_for_blue() if action.target else None,
                    group=action.group,
                    motion=action.motion,
                    points=action.points,
                    reason=action.reason,
                    extra_params=action.extra_params.copy()
                )
                actions.append(new_action)
        
        # Add +90° for coordinate system alignment
        init_pose = Position(init_pose.x, init_pose.y, init_pose.theta_deg + 90.0)
        home_pose = Position(home_pose.x, home_pose.y, home_pose.theta_deg + 90.0)
        
        result = {
            "init_pose": init_pose.to_dict(),
            "home_pose": home_pose.to_dict(),
            "actions": [action.to_dict() for action in actions]
        }
        
        # Add group information for action cancellation
        if self.groups:
            result["groups"] = {
                name: {
                    "action_count": len(group.actions)
                }
                for name, group in self.groups.items()
            }
        
        return result
    
    def get_actions_by_group(self, group_name: str) -> List[Action]:
        """Get all actions belonging to a specific group"""
        return [action for action in self.actions if action.group == group_name]
    
    def get_group_names(self) -> List[str]:
        """Get all group names"""
        return list(self.groups.keys())

# Utility functions for creating common strategies
def create_element_collection_strategy(start_pos: Position, element_positions: List[Position], 
                                     drop_positions: List[Position]) -> StrategyBuilder:
    """Create a generic element collection strategy"""
    strategy = StrategyBuilder()
    
    for i, (element_pos, drop_pos) in enumerate(zip(element_positions, drop_positions)):
        group_name = f"elements_{i+1}"
        strategy.create_group(group_name)
        strategy.take_elements_sequence(element_pos.x, element_pos.y, element_pos.theta_deg, group_name)
        strategy.put_elements_sequence(drop_pos.x, drop_pos.y, drop_pos.theta_deg, group_name)
    
    return strategy
