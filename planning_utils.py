import sys
from os.path import dirname, abspath
from enum import Enum
from queue import PriorityQueue
import numpy as np
import re
from math import sqrt
from udacidrone.frame_utils import global_to_local

from planning import a_star,a_star_graph_complete
from simpleAction import ActionCity,valid_actions_city
from heuristicFunctions import EculdianDistance,NormalDistance
from Collinearity import Collinear
from grid_to_graph import closest_point,create_graph_from_edges,create_grid_and_edges_voronoi


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


def find_path_grid(grid, grid_start, grid_goal):

    # Run A* to find a path from start to goal
    # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
    path, _ = a_star(grid, grid_start, grid_goal, EculdianDistance, ActionCity, valid_actions_city)

    # TODO: prune path to minimize number of waypoints
    path = Collinear.prune_path(path)
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
    path = Collinear.prune_path(path)
    return path