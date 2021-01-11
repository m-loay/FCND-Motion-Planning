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
from sklearn.neighbors import KDTree
from ConectNodes import create_graph
from ConfgurationSpace import readData
from ConfgurationSpace import create_grid
from Sampling import Sampler
from heuristicFunctions import NormalDistance
from heuristicFunctions import EculdianDistance
from planning import a_star
from planning import a_star_graph_complete
from ReConstructPath import retrivePathWithCost
from simpleAction import ActionCity
from simpleAction import valid_actions_city
from shapely.geometry import Polygon, Point, LineString
from voxel_map import closest_point_index
from voxel_map import closest_point

#from bresenham import bresenham
plt.rcParams['figure.figsize'] = 12, 12

########################################################################################################################
# Step 1 - Load Data
########################################################################################################################
# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
data = readData()

########################################################################################################################
# Step 2 - Sample Points 
########################################################################################################################
sampler = Sampler(data)
polygons = sampler._polygons
nodes = sampler.sample(300)
print(len(nodes))
print(len(polygons))

########################################################################################################################
# Step 3 - Connect Nodes
########################################################################################################################
g = create_graph(nodes, 10, polygons)

########################################################################################################################
# Step 4 - Visualize Graph
########################################################################################################################
grid = create_grid(data, 10, 1)
fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black' , alpha=0.5)

# draw all nodes
for n1 in nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')
    
# draw connected nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    
plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()

########################################################################################################################
# Step 5: Path Planing using A*
########################################################################################################################
#set start and goal
# start = list(g.nodes)[0]
# k = np.random.randint(len(g.nodes))
# print(k, len(g.nodes))
# goal = list(g.nodes)[k]
start_ne = (25,  100,0)
goal_ne = (750., 370.,0)
# start_index = closest_point_index(g, start_ne)
# goal_index =  closest_point_index(g, goal_ne)
# print("index")
# print(start_index)
# print(goal_index)
# start = list(g.nodes)[start_index]
# goal = list(g.nodes)[goal_index]
start =  closest_point(g, start_ne)
goal =  closest_point(g, goal_ne)
print("Points")
print(start)
print(goal)
#Using voronoi to plan path
branch = a_star_graph_complete(g, start, goal, NormalDistance,ActionCity, valid_actions_city)
path, path_cost = retrivePathWithCost(start, goal, branch)
print("Path length = {0}, path cost = {1}".format(len(path), path_cost))

path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    print(n1, n2)

########################################################################################################################
# Step 6: Visualize Path
########################################################################################################################
fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    
# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')
    
# TODO: add code to visualize the path
path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')


plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()
