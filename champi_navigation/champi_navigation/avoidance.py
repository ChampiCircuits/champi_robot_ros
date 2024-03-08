# lib pour l'evitement d'obstacle grace a la methode du graphe de visibilité

from champi_navigation.world_state import WorldState
from champi_navigation.math_bind import *

from dijkstar import Graph, find_path
from shapely import Point, Polygon, LineString, intersection
from icecream import ic


def create_graph(start: Point, goal: Point, world_state:WorldState):
    ic.disable()
    """Create the graph of navigation from is point to the goal

    Args:
        start (Point): The start point of the robot (robot pos)
        goal (Point): The goal point of the robot
        world_state (WorldState): The instance of the world state
    """

    ic("CREATING GRAPH\n\n")

    expanded_table_poly = world_state.get_table().expanded_poly
    expanded_opponent_robot_poly = world_state.get_opponent_robot().expanded_poly


    # check if goal lies inside the obstacle
    if point_inside_poly(goal, expanded_opponent_robot_poly):
        ic("GOAL INSIDE OPPONENT ROBOT")
        return Graph(), {}
    # check if goal lies outside the table
    if not point_inside_poly(goal, expanded_table_poly):
        ic("GOAL OUTSIDE TABLE")
        return Graph(), {}
    
    graph = Graph()

    # create a dictionnary associating each point with an index
    dico_all_points = {}
    dico_all_points[len(dico_all_points)] = (start.x,start.y)
    dico_all_points[len(dico_all_points)] = (goal.x,goal.y)
    for point in list(expanded_opponent_robot_poly.exterior.coords):
        dico_all_points[len(dico_all_points)] = point

    ic("DICO ALL POINTS CREATED")
    

# GENERATING COMBINATIONS
    # generate each combination between start/goal to every other points to test visibility
    all_combinations = []
    for key in dico_all_points.keys():
        if key != 0 and key != 1: # start and goal
            all_combinations.append((key, 0))
            all_combinations.append((key, 1))

    # Add the segment between start and goal as a combination to test
    all_combinations.append((0,1)) 

    ic("COMBINATIONS CREATED")

# ADDING EDGES TO THE GRAPH
    # generate the edges of the polygon that should not be crossed
    # Todo TROUVER UNE AUTRE METHODE
    obstacle_edges = []
    for i in range(len(dico_all_points)-2):
        if i==len(dico_all_points)-2-1:
            obstacle_edges.append((i+2, 2))
        else:
            obstacle_edges.append((i+2, i+2+1))

    ic("OBSTACLE EDGES CREATED")

    # Add the edges of the polygon to the graph as they are admissible by nature
    for seg in obstacle_edges:
        d = dist(dico_all_points[seg[0]],dico_all_points[seg[1]])
        graph.add_edge(seg[0],seg[1],d)
        graph.add_edge(seg[1],seg[0],d)

    edges_to_not_cross = []
    for edge in get_edges(expanded_opponent_robot_poly):
        edges_to_not_cross.append(edge)
    for edge in get_edges(expanded_table_poly):
        edges_to_not_cross.append(edge)

    ic("EDGES TO NOT CROSS CREATED")

    # check for each segment/combination of two points if they cross the obstacle
    # if not we add them as edges to the graph
    for comb in all_combinations:
        a, b = comb
        pointA = dico_all_points[a]
        pointB = dico_all_points[b]
        segment = LineString([pointA, pointB])

        # Check if there is an intersection with one obstacle or the table
        no_inter = True
        for edge in edges_to_not_cross:
            inter = intersection(edge, segment) # TODO vor si il y a un probleme ici
            if inter == None or inter.is_empty:
                continue
            
            inter = point_to_tuple(inter)
            edgeB = edge.coords[1]
            edgeA = edge.coords[0]
            
            # if segments have a common point, having an intersection is OK
            if not eq_tuples(pointA, inter) and not eq_tuples(inter, pointB) and not eq_tuples(inter, edgeA) and not eq_tuples(inter,edgeB):
                no_inter = False
                break

        if no_inter:
            d = dist(dico_all_points[a],dico_all_points[b])
            graph.add_edge(a,b,d)
            graph.add_edge(b,a,d)
            # ic(
            #     str(pointA[0])+","+str(pointA[1]),
            #     str(pointB[0])+","+str(pointB[1])
            # )
            # print()


    ic("CREATED GRAPH\n\n")
    ic.enable()

    return graph, dico_all_points



def find_avoidance_path(graph, start, end):
    """Find a path between the start and the end with the given graph if possible

    Args:
        graph (Graph): The graph of visibility
        start (Point): The start point
        end (Point): The goal point

    Returns:
        list[int]: The list of the indices of the points of the path
    """
    try:
        return find_path(graph, start, end)
    except:
        return None

def get_edges(poly: Polygon):
    """Get the edges of a polygon

    Args:
        poly (Polygon): The polygon to get the edges from

    Returns:
        list[LineString]: The list of edges
    """
    edges = []
    for i in range(len(poly.exterior.coords)-1):
        edges.append(LineString([poly.exterior.coords[i],poly.exterior.coords[i+1]]))
    return edges