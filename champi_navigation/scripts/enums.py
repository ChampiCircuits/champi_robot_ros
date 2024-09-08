from enum import Enum

class TolerancesMeters(Enum):
    TIGHT = 0.005   # 5mm
    MEAN  = 0.01    # 1cm
    LARGE = 0.05    # 5cm

class PointType(Enum):
    ENDPOINT = 0
    WAYPOINT = 1
