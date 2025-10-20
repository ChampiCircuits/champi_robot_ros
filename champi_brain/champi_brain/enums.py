
from typing import Optional
from enum import Enum

class ZoneType(Enum):
    NOT_IN_A_ZONE = "not_in_a_zone"     # Default type when no zone is assigned, so elements are free to be taken
    PLACEMENT_ZONE = "placement_zone"   # Zone where elements can be placed, but also stolen by opponent
    SECURE_ZONE = "secure_zone"         # Zone where elements are safe from opponent

class Color(Enum):
    YELLOW = "yellow"
    BLUE = "blue"
    NOT_INITIALIZED = "not_initialized"

class ElementState(Enum):
    ON_TABLE = "on_table"   # Initial state, box is on the table
    TAKEN = "taken"         # Box taken by our robot
    PLACED = "placed"       # Box placed on the table
    SECURED = "secured"     # Box placed where the opponent cannot take it
    MISSING = "missing"     # Box no longer at expected location

class GameElement:
    def __init__(self, id: str, x: float, y: float, theta_deg: float, state: ElementState):
        self.id: str = id
        self.x: float = x
        self.y: float = y
        self.theta_deg: float = theta_deg
        self.state: ElementState = state
        self.missing_count: int = 0  # Number of consecutive times this element was not observed

class NutsBox(GameElement):
    def __init__(self, id: str, x: float, y: float, theta_deg: float, color: Color, state: ElementState):
        super().__init__(id, x, y, theta_deg, state)
        self.color: Color = color

class Zone():
    def __init__(self, id: str, x: float, y: float, width: float, height: float, zone_type:ZoneType, color: Optional[Color] = None):
        self.id: str = id
        self.x: float = x
        self.y: float = y
        self.width: float = width
        self.height: float = height
        self.type: ZoneType = zone_type
        self.color: Optional[Color] = color # Optional color for secure zone

