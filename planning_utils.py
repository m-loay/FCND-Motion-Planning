from queue import PriorityQueue
from enum import Enum
import numpy as np
import numpy.linalg as LA
from scipy.spatial import Voronoi
from bresenham import bresenham
import networkx as nx
from udacidrone.frame_utils import global_to_local
import re

def read_data(filename):
    """
    Reads home (lat, lon) from the first line of the `file`.
    """
    with open(filename) as f:
        first_line = f.readline()
    match = re.match(r'^lat0 (.*), lon0 (.*)$', first_line)
    if match:
        lat = match.group(1)
        lon = match.group(2)
    
    return np.fromstring(f'{lat},{lon}', dtype='Float64', sep=',')

def create_grid(data, drone_altitude =5, safety_distance=3):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def create_grid_and_edges_voronoi(data, drone_altitude=5, safety_distance=3):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, int(north_min), int(east_min)

def create_graph_from_edges(edges):
    """
    Create a graph from the `edges`
    """
    G = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        dist = LA.norm(np.array(p2) - np.array(p1))
        G.add_edge(p1, p2, weight=dist)
    return G

def closest_point(graph, point):
    """
    Compute the closest point in the `graph`
    to the `point_3d`.
    """
    current_point = (point[0], point[1])
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def calculate_start_goal_local_position(global_start, global_goal, global_home, north_offset, east_offset):
    #concatenate offset

    # TODO: retrieve current global position
    # TODO: convert to current local position using global_to_local()
    # TODO: convert start position to current position rather than map center
    
    #get start grid Position
    local_north, local_east, local_down = global_to_local(global_start, global_home)
    grid_start = (int(np.ceil(local_north - north_offset)), int(np.ceil(local_east - east_offset)))

    #get goal grid Position
    goal_north, goal_east, goal_alt = global_to_local(global_goal, global_home)
    grid_goal = (int(np.ceil(goal_north - north_offset)), int(np.ceil(goal_east - east_offset)))
    
    return grid_start,grid_goal

# Quadroter assume all actions cost the same.
class ActionCity(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

def valid_actions_city(grid, current_node,Action=ActionCity):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions

def EculdianDistance(position, goal_position):
    h = np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
    return h

def NormalDistance(n1,n2):
    return LA.norm(np.array(n2) - np.array(n1))

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check_int(p11, p22, p33): 
    collinear = False
    p1 = np.array([p11[0][0], p11[0][1]])
    p2 = np.array([p22[0][0], p22[0][1]])
    p3 = np.array([p33[0][0], p33[0][1]])
    # TODO: Calculate the determinant of the matrix using integer arithmetic
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1]) 
    # TODO: Set collinear to True if the determinant is equal to zero
    if det == 0:
        collinear = True
    return collinear

def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check_int(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def retrivePathWithCost(start, goal, branch):
    # Retrace your steps
    path = []
    n = goal
    path.append(goal)
    path_cost = branch[n][0]
    while branch[n][1] != start:
        # Append each new node to the path as you work your way back
        path.append(branch[n][1])
        n = branch[n][1]
    # One last time to append the start location
    path.append(branch[n][1])

    # And reverse the order to make it a path from start to goal
    return path[::-1],path_cost

def a_star(grid, start, goal, h, Ac, valid_action):
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    path = []
    path_cost = 0
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for a in valid_action(grid, current_node, Ac):
                # get the tuple representation
                da = a.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: calculate branch cost (action.cost + g)
                branch_cost = current_cost + a.cost
                # TODO: calculate queue cost (action.cost + g + h)
                queue_cost = branch_cost + h(next_node,goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, a)
                    queue.put((queue_cost, next_node))
    if(found):
        path,path_cost = retrivePathWithCost(start, goal, branch)
    else:
        print("path Not Found")

    return path,path_cost

def a_star_graph_complete(graph, start, goal, h,Ac, valid_action):
    """Modified A* to work with NetworkX graphs."""
    
    # TODO: complete
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
    if(found):
        path,path_cost = retrivePathWithCost(start, goal, branch)
    else:
        print("path Not Found")

    return path,path_cost

def find_path_grid(grid, grid_start, grid_goal):

    # Run A* to find a path from start to goal
    # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
    path, _ = a_star(grid, grid_start, grid_goal, EculdianDistance, ActionCity, valid_actions_city)

    # TODO: prune path to minimize number of waypoints
    path = prune_path(path)
    return path

def find_path_graph_voronoi(graph, grid_start, grid_goal):
    ##use graph to get closet start and goal point
    start_ne_g = closest_point(graph, grid_start)
    goal_ne_g = closest_point(graph, grid_goal)
    # Run A* to find a path from start to goal
    # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation

    path, _ = a_star_graph_complete(graph, start_ne_g, goal_ne_g, EculdianDistance, ActionCity, valid_actions_city)
    path.append(grid_goal)

    # TODO: prune path to minimize number of waypoints
    path = prune_path(path)
    return path