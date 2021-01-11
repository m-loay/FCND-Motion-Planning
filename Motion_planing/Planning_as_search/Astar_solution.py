#!/usr/bin/env python
# coding: utf-8

# # Breadth-First Search with cost

# In this exercise you'll implement the breadth-first search algorithm.
# Import numpy, Enum and Queue
from PathVisualize import visualize_path
from planning import a_star
from heuristicFunctions import EculdianDistance
from simpleAction import valid_actions
from ReConstructPath import retrivePathActions
import numpy as np

# https://wiki.python.org/moin/TimeComplexity gives a solid overview of Python data
# structures and their time complexity.

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
branch = a_star(grid, start, goal,EculdianDistance)
path, path_cost = retrivePathActions(start, goal, branch)
print(path)
print(path_cost)

# S -> start, G -> goal, O -> obstacle
print(visualize_path(grid, path, start))