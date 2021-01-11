
def retrivePath(start, goal, branch):
    # Retrace your steps
    path = []
    n = goal
    while branch[n][0] != start:
        # Append each new node to the path as you work your way back
        path.append(branch[n][1])
        n = branch[n][0]
    # One last time to append the start location
    path.append(branch[n][1])

    # And reverse the order to make it a path from start to goal
    return path[::-1]

def retrivePathWithCost(start, goal, branch):
    # Retrace your steps
    path = []
    n = goal
    # path.append(goal)
    path_cost = branch[n][0]
    while branch[n][1] != start:
        # Append each new node to the path as you work your way back
        path.append(branch[n][1])
        n = branch[n][1]
    # One last time to append the start location
    path.append(branch[n][1])

    # And reverse the order to make it a path from start to goal
    return path[::-1],path_cost

def retrivePathActions(start, goal, branch):
    # Retrace your steps
    path = []
    n = goal
    path_cost = branch[n][0]
    while branch[n][1] != start:
        # Append each new node to the path as you work your way back
        path.append(branch[n][2])
        n = branch[n][1]
    # One last time to append the start location
    path.append(branch[n][2])

    # And reverse the order to make it a path from start to goal
    return path[::-1],path_cost