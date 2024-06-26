# lib pour l'evitement d'obstacle grace a la methode du graphe de visibilité

from champi_navigation.world_state import WorldState
from champi_navigation.math_bind import *

from dijkstar import Graph, find_path
from shapely import Point, Polygon, LineString, intersection
from icecream import ic


def create_graph(start: Point, goal: Point, world_state:WorldState):
    """Create the graph of navigation from is point to the goal

    Args:
        start (Point): The start point of the robot (robot pos)
        goal (Point): The goal point of the robot
        world_state (WorldState): The instance of the world state
    """

    # ic("CREATING GRAPH\n\n")

    expanded_table_poly = world_state.table.expanded_poly
    expanded_opponent_robot_poly = world_state.opponent_robot.expanded_poly
    expanded_opponent_robot_poly2 = world_state.opponent_robot.expanded_poly2


    # check if goal lies inside the obstacle
    if point_inside_poly(goal, expanded_opponent_robot_poly):
        ic("GOAL INSIDE OPPONENT ROBOT")
        return Graph(), {}
    # check if goal lies outside the table
    if not point_inside_poly(goal, expanded_table_poly):
        ic("GOAL OUTSIDE TABLE")
        return Graph(), {}
    
    graph = Graph(undirected=True)

    # create a dictionnary associating each point with an index
    dico_all_points = {}
    dico_all_points["0"] = (start.x,start.y)
    dico_all_points["1"] = (goal.x,goal.y)
    for point in list(expanded_opponent_robot_poly2.exterior.coords):
        dico_all_points[str(len(dico_all_points))] = point

    # ic("DICO ALL POINTS CREATED")
    # ic(dico_all_points)

    for i in range(len(expanded_table_poly.exterior.coords)+2):
        graph.add_node(str(i))
    

# GENERATING COMBINATIONS
    # generate each combination between start/goal to every other points to test visibility with
    all_combinations = []
    for key in dico_all_points.keys():
        if key != "0" and key != "1": # start and goal
            all_combinations.append((key, "0"))
            all_combinations.append((key, "1"))

    # Add the segment between start and goal as a combination to test
    all_combinations.append(("0","1")) 

    # ic("COMBINATIONS CREATED")

# ADDING EDGES TO THE GRAPH
    obstacle_edges = []
    for i in range(len(expanded_opponent_robot_poly2.exterior.coords)-1):
        obstacle_edges.append((i+2,i+1+2)) # because 0 and 1 are start and goal
    obstacle_edges.append((len(expanded_opponent_robot_poly2.exterior.coords)-1+2,2))

    # ic("OBSTACLE EDGES CREATED")

    # Add the edges of the polygon to the graph as they are admissible by nature
    for seg in obstacle_edges:
        d = dist(dico_all_points[str(seg[0])],dico_all_points[str(seg[1])])
        graph.add_edge(str(seg[0]), str(seg[1]), d)

    # generate the edges of the polygon that should not be crossed
    edges_to_not_cross = []
    for edge in get_edges(expanded_opponent_robot_poly):
        edges_to_not_cross.append(edge)
    for edge in get_edges(expanded_table_poly):
        edges_to_not_cross.append(edge)

    # ic("EDGES TO NOT CROSS CREATED")

    # check for each segment/combination of two points if they cross edges_to_not_cross
    # if not we add them as edges to the graph
    # print()
    # print()
    # print()
    # print()
    for comb in all_combinations:
        a, b = comb
        # print(comb)
        pointA = dico_all_points[str(a)]
        pointB = dico_all_points[str(b)]
        segment = LineString([pointA, pointB])

        # Check if there is an intersection with one obstacle or the table
        no_inter = True
        for edge in edges_to_not_cross:
            inter = intersection(edge, segment) # TODO vor si il y a un probleme ici
            if inter == None or inter.is_empty:
                continue
            
            #to tuple
            inter = (inter.x, inter.y)
            if eq_tuples(inter, pointA) or eq_tuples(inter, pointB):
                continue
            
            no_inter = False
            break

        if no_inter:
            d = dist(dico_all_points[str(a)],dico_all_points[str(b)])
            graph.add_edge(a,b,d)

    # print()
    # print(graph["0"], graph["1"])
    # print()

    return graph, dico_all_points



def find_avoidance_path_(graph, start, end):
    """Find a path between the start and the end with the given graph if possible

    Args:
        graph (Graph): The graph of visibility
        start (Point): The start point
        end (Point): The goal point

    Returns:
        list[int]: The list of the indices of the points of the path
    """
    try:
        # print("TRYING TO FIND A PATH BETWEEN",type(start),type(end),start,end)
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
    edges.append(LineString([poly.exterior.coords[-1],poly.exterior.coords[0]]))
    return edges