import numpy as np
import numpy.linalg as LA
from math import sqrt
# TODO: implement a heuristic function. This may be one of the
# functions described above or feel free something to think of something
# else.
def EculdianDistance(position, goal_position):
    h = np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
    return h

def NormalDistance(n1,n2):
    return LA.norm(np.array(n2) - np.array(n1))
