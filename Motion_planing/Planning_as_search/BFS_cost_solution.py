#!/usr/bin/env python
# coding: utf-8

# # Breadth-First Search with cost

# In this exercise you'll implement the breadth-first search algorithm.
# Import numpy, Enum and Queue
from PathVisualize import visualize_path
from planning import breadth_first_cost
import numpy as np

# https://wiki.python.org/moin/TimeComplexity gives a solid overview of Python data structures and their time complexity.

# Define a start and goal location
start = (0, 0)
goal = (4, 4)

# Define your grid-based state space of obstacles and free space
grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

# ### Executing the search
# 
# Run `breadth_first()` and reference the grid to see if the path makes sense.

path, path_cost = breadth_first_cost(grid, start, goal)
print(path)
print(path_cost)

# S -> start, G -> goal, O -> obstacle
print(visualize_path(grid, path, start))




