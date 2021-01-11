'''
Finding Your Way In The City¶
In this notebook you'll combine the work of previous exercises to calculate a minimal series of waypoints in order to get from a start location to a goal location.

You'll reuse and modify your algorithms from:

A*
Configuration Space
Collinearity and/or Bresenham
'''
import sys
from os.path import dirname, abspath
d = dirname(dirname(abspath(__file__)))
Flying_Car_Representation = d + '\\Flying_Car_Representation'
Planning_as_search = d + '\\Planning_as_search'
sys.path.insert(1, Flying_Car_Representation)
sys.path.insert(1, Planning_as_search)

import numpy as np
import matplotlib.pyplot as plt
from Collinearity import Collinear
from ConfgurationSpace import readData
from ConfgurationSpace import create_grid
from heuristicFunctions import EculdianDistance
from planning import a_star
from ReConstructPath import retrivePathWithCost
from simpleAction import ActionCity
from simpleAction import valid_actions_city

#from bresenham import bresenham
plt.rcParams['figure.figsize'] = 12, 12

'''
You'll notice we've imported create_grid, and a_star.
These are functions you've implemented in previous exercises,
and here you'll use them to create a map and find a path from a starting position to a goal position.

To read the function signature and documentation execute ? 
followed by the function name in a cell. In the example below we'll check the documentation for create_grid.
'''
## This is the same obstacle data from the previous lesson.
#create a grid
# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3

# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
data = readData()
grid = create_grid(data, drone_altitude, safe_distance)
# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

'''
Next you'll compute the path from a start location to a goal location using A*.

Start and goal coordinates in (north, east).
'''
start_ne = (25,  100)
goal_ne = (255., 900.)
# TODO: use `a_star` to compute the lowest cost path
branch = a_star(grid, start_ne, goal_ne,EculdianDistance,ActionCity,valid_actions_city)
path, path_cost = retrivePathWithCost(start_ne, goal_ne, branch)
print(len(path), path_cost)

'''
Let's plot the path!
'''
plt.imshow(grid, cmap='Greys', origin='lower')

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')
pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'g')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

##Path Pruning
path = Collinear.prune_path(path)
print(len(path), path_cost)
plt.imshow(grid, cmap='Greys', origin='lower')

plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

if path is not None:
    pp = np.array(path)
    plt.plot(pp[:, 1], pp[:, 0], 'g')
    plt.scatter(pp[:, 1], pp[:, 0])

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()