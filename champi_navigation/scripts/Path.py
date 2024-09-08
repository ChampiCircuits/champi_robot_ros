from geometry_msgs.msg import Pose

from typing import Optional
from enum import Enum

class TolerancesMeters(Enum):
    TIGHT = 0.005   # 5mm
    MEAN  = 0.01    # 1cm
    LARGE = 0.05    # 5cm

class PointType(Enum):
    ENDPOINT = 0
    WAYPOINT = 1

class Point(Pose):
    def __init__(self,
                 name: str,
                 pose: Pose, 
                 point_type: PointType,
                 tolerance:TolerancesMeters,
                 robot_should_stop_here:bool = False):
        
        super().__init__()
        
        self.name = name

        self.position = pose.position
        self.orientation = pose.orientation

        self.point_type = point_type
        self.tolerance = tolerance
        self.robot_should_stop_here = robot_should_stop_here

    def __repr__(self):
        return (f"Point(name={self.name}, "
                f"point_type=({self.point_type.name}), "
                f"tolerance=({self.tolerance.name}), "
                f"robot_should_stop_here=({self.robot_should_stop_here}), "
                f"position=({self.position.x}, {self.position.y}, {self.position.z}), "
                f"orientation=({self.orientation.x}, {self.orientation.y}, {self.orientation.z}, {self.orientation.w}))")


class Segment():
    def __init__(self,
                 name: str,
                 start: Point, 
                 end:Point, 
                 speed:float,
                 look_at_point:Optional[Point] = None):
        
        self.name = name
        self.start = start
        self.end   = end
        self.speed = speed
        self.look_at_point = look_at_point

    def __repr__(self):
        return (f"Segment(name={self.name}, "
                f"speed=({self.speed}), "
                f"look_at_point=({self.look_at_point}), "
                f"\n\tstart=({self.start}), "
                f"\n\tend=({self.end}))")


class Path():
    def __init__(self,
                 name: str,
                 segments: list[Segment]):
        
        self.name = name
        self.segments = segments

    def __repr__(self):
        segments_string = ""
        for seg in self.segments:
            segments_string += "\n - " + seg.__repr__()
        return (f"Path(name=({self.name}), "
                f"segments=({segments_string}))")
    

if __name__ == "__main__":
    a = Point("a", Pose(), PointType.ENDPOINT, TolerancesMeters.LARGE, False)
    b = Point("b", Pose(), PointType.ENDPOINT, TolerancesMeters.LARGE, False)

    seg = Segment("seg",a,b,0.1)


    path = Path("p", [seg,seg])
    print(path)