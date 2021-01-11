from enum import Enum
from queue import PriorityQueue
import numpy as np
import sys
from os.path import dirname, abspath
d = dirname(dirname(abspath(__file__)))
Flying_Car_Representation = d + '\\Flying_Car_Representation'
Planning_as_search = d + '\\Planning_as_search'
sys.path.insert(1, Flying_Car_Representation)
sys.path.insert(1, Planning_as_search)
from simpleAction import ActionCity
from simpleAction import valid_actions_city
from planning import a_star

def a_starCity(grid, h, start, goal):
    path = []
    path_cost = 0
    branch = a_star(grid, start, goal,h,ActionCity,valid_actions_city)
    return branch


# def a_starCity(grid, h, start, goal):

#     path = []
#     path_cost = 0
#     queue = PriorityQueue()
#     queue.put((0, start))
#     visited = set(start)

#     branch = {}
#     found = False
    
#     while not queue.empty():
#         item = queue.get()
#         current_node = item[1]
#         if current_node == start:
#             current_cost = 0.0
#         else:              
#             current_cost = branch[current_node][0]
            
#         if current_node == goal:        
#             print('Found a path.')
#             found = True
#             break
#         else:
#             for action in valid_actions_city(grid, current_node,ActionCity):
#                 # get the tuple representation
#                 da = action.delta
#                 next_node = (current_node[0] + da[0], current_node[1] + da[1])
#                 branch_cost = current_cost + action.cost
#                 queue_cost = branch_cost + h(next_node, goal)
                
#                 if next_node not in visited:                
#                     visited.add(next_node)               
#                     branch[next_node] = (branch_cost, current_node, action)
#                     queue.put((queue_cost, next_node))
             
#     if found:
#         # retrace steps
#         n = goal
#         path_cost = branch[n][0]
#         path.append(goal)
#         while branch[n][1] != start:
#             path.append(branch[n][1])
#             n = branch[n][1]
#         path.append(branch[n][1])
#     else:
#         print('**********************')
#         print('Failed to find a path!')
#         print('**********************') 
#     return path[::-1], path_cost