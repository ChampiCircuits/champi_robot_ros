from dataclasses import dataclass
from typing import Optional
from enum import Enum

###################################
### ENUMS
###################################
class ZoneType(Enum):
    NOT_IN_A_ZONE = "not_in_a_zone"     # Default type when no zone is assigned, so elements are free to be taken
    PLACEMENT_ZONE = "placement_zone"   # Zone where elements can be placed, but also stolen by opponent
    SECURE_ZONE = "secure_zone"         # Zone where elements are safe from opponent
    FORBIDDEN_ZONE = "forbidden_zone"   # Zone where robot shall not enter

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

###################################
### DATA CLASSES
###################################
@dataclass
class GameElement:
    """Base class for game elements on the table."""
    id: str
    x: float
    y: float
    theta_deg: float
    state: ElementState
    missing_count: int = 0  # Number of consecutive times this element was not observed

@dataclass
class NutsBox(GameElement):
    """A nuts box element with a specific color."""
    color: Color = Color.NOT_INITIALIZED

@dataclass
class Zone:
    """A zone on the table (placement, secure, or forbidden)."""
    id: str
    x: float
    y: float
    width: float
    height: float
    zone_type: ZoneType
    color: Optional[Color] = None  # Optional color for secure zone

