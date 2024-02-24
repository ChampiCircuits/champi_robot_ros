from math import *
from shapely import Point, Polygon, LineString, BufferCapStyle

def expand(poly: Polygon, offset: float) -> Polygon:
    """Expand a polygon by a given offset and return the expanded polygon"""
    print(len(poly.exterior.coords))
    return poly.buffer(offset,cap_style="flat", resolution=1)

def eq_tuples(pointA: tuple, pointB: tuple) -> bool:
    """Check if two tuples of points are almost equals at the 2nd decimal"""
    return round(float(pointA[0]), 2) == round(float(pointB[0]), 2) and round(float(pointA[1]), 2) == round(float(pointB[1]), 2) 

def eq_points(pointA: Point, pointB: Point) -> bool:
    """Check if two points are almost equals at the 2nd decimal"""
    return pointA.equals_exact(pointB,1e-2)

def eq_segs(segA: LineString, segB: LineString) -> bool:
    """Check if two segments are almost equals at the 2nd decimal"""
    return segA.equals_exact(segB,1e-2)

def dist(A: Point,B: Point) -> float:
    """Compute the euclidian distance between two points"""
    return sqrt(pow(B[0]-A[0],2)+pow(B[1]-A[1],2))

def point_to_tuple(p: Point) -> tuple:
    """Transform a Point to a tuple of coordinates"""
    return (p.x,p.y)

def tuple_to_point(t: tuple) -> Point:
    """Transform a tuple of coordinates to a Point"""
    return Point(t[0],t[1])

def point_inside_poly(p: Point, poly:Polygon):
    return poly.contains(p) 