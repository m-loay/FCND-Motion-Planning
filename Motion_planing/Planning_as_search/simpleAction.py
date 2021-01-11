
# [`Enum`](https://docs.python.org/3/library/enum.html#module-enum) is used to represent possible actions on the grid.
from enum import Enum
import numpy as np 
# Define your action set using Enum()
class Action(Enum): 
    # Actions are tuples corresponding to movements in (i, j)
    LEFT = (0, -1)
    RIGHT = (0, 1)
    UP = (-1, 0)
    DOWN = (1, 0)
    
    # Define string characters for each action
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
            
class ActionCost(Enum):
    # Assign the cost of each action as the third element in the tuple
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
    # Assign a new property that returns the cost of an action
    @property
    def cost(self):
        return self.value[2]
    # Assign a property that returns the action itself
    @property
    def delta(self):
        return (self.value[0], self.value[1])

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

# Define a function that returns a list of valid actions 
# through the grid from the current node
def valid_actions(grid, current_node,AC=Action):
    """
    Returns a list of valid actions given a grid and current node.
    """
    # First define a list of all possible actions
    valid = [AC.UP, AC.LEFT, AC.RIGHT, AC.DOWN]
    # Retrieve the grid shape and position of the current node
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or it's an obstacle
    # If it is either, remove the action that takes you there
    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(AC.UP)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(AC.DOWN)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(AC.LEFT)
    if y + 1 > m or grid[x, y+1] == 1:
        valid.remove(AC.RIGHT)
        
    return valid

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