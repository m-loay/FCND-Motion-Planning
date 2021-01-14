'''
Finding Your Way In The City¶
In this notebook you'll combine the work of previous exercises to calculate 
a minimal series of waypoints in order to get from a start location to a goal location.

You'll reuse and modify your algorithms from:

A*
Configuration Space
Collinearity and/or Bresenham
'''
########################################################################################################################
# Import Section
########################################################################################################################
import sys
from os.path import dirname, abspath
d = dirname(dirname(abspath(__file__)))
Flying_Car_Representation = d + '\\Flying_Car_Representation'
Planning_as_search = d + '\\Planning_as_search'
sys.path.insert(1, Flying_Car_Representation)
sys.path.insert(1, Planning_as_search)

import numpy as np
import numpy.linalg as LA
import networkx as nx
import matplotlib.pyplot as plt
from ConfgurationSpace import readData
from grid_to_graph import create_grid_and_edges_voronoi
from grid_to_graph import create_graph_from_edges
from grid_to_graph import closest_point
from Collinearity import Collinear
from heuristicFunctions import NormalDistance
from heuristicFunctions import EculdianDistance
from planning import a_star
from planning import a_star_graph_complete
from ReConstructPath import retrivePathWithCost
from simpleAction import ActionCity
from simpleAction import valid_actions_city

#from bresenham import bresenham
plt.rcParams['figure.figsize'] = 12, 12

########################################################################################################################
# Step 1: Create Grid and Edges based on drone altitude and safety distance
########################################################################################################################
## This is the same obstacle data from the previous lesson.
#create a grid
# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3

# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
data = readData()
grid, edges, _, _ = create_grid_and_edges_voronoi(data,drone_altitude, safe_distance)


########################################################################################################################
# Step 2: Define start and Goal 
########################################################################################################################
start_ne = (25,  100)
goal_ne = (750., 370.)

##Draw Grid with new start and goal
plt.imshow(grid, origin='lower') 
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')
plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

########################################################################################################################
# Step 2: Create the voronoi and find new start and goal
########################################################################################################################
# TODO: create the graph with the weight of the edges
# set to the Euclidean distance between the points
G = create_graph_from_edges(edges)

##use graph to get closet start and goal point
start_ne_g = closest_point(G, start_ne)
goal_ne_g = closest_point(G, goal_ne)

##Draw skelton with new start and goal
# equivalent to
# plt.imshow(np.flip(grid, 0))
# Plot it up!
plt.imshow(grid, origin='lower', cmap='Greys')
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')
plt.plot(start_ne_g[1], start_ne_g[0], 'gx')
plt.plot(goal_ne_g[1], goal_ne_g[0], 'gx')

# Stepping through each edge
for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

########################################################################################################################
# Step 3: Plan path using A*
########################################################################################################################

#Using Normal Grid to plan path
path, path_cost = a_star(grid, start_ne, goal_ne,EculdianDistance,ActionCity,valid_actions_city)
print("RED Grid Path length = {0}, path cost = {1}".format(len(path), path_cost))

#Using voronoi to plan path
path2, path_cost2 = a_star_graph_complete(G, start_ne_g, goal_ne_g, EculdianDistance,ActionCity, valid_actions_city)
print("Green Voronoi Path length = {0}, path cost = {1}".format(len(path2), path_cost2))

#Draw Path on Grid and Skelton
plt.imshow(grid, cmap='Greys', origin='lower')
# plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')
plt.plot(start_ne_g[1], start_ne_g[0], 'gx')
plt.plot(goal_ne_g[1], goal_ne_g[0], 'gx')
pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'r')
pp2 = np.array(path2)
plt.plot(pp2[:, 1], pp2[:, 0], 'g')
plt.show()


########################################################################################################################
# Step 4: Path Pruning
########################################################################################################################

#Prune the A* path
path = Collinear.prune_path(path)
print("RED Pruned Grid Path length = {0}, path cost = {1}".format(len(path), path_cost))

path2 = Collinear.prune_path(path2)
print("Green Pruned Skelton Path length = {0}, path cost = {1}".format(len(path2), path_cost2))

#Draw Prunned Path on Grid and Skelton
plt.imshow(grid, cmap='Greys', origin='lower')
# plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'r')
plt.scatter(pp[:, 1], pp[:, 0])

pp2 = np.array(path2)
plt.plot(pp2[:, 1], pp2[:, 0], 'g')
plt.scatter(pp2[:, 1], pp2[:, 0])

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()