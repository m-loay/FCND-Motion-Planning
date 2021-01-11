
from queue import Queue
from queue import PriorityQueue
from simpleAction import Action
from simpleAction import ActionCost
from simpleAction import ActionCity
from simpleAction import valid_actions
from ReConstructPath import retrivePath
from ReConstructPath import retrivePathWithCost
from heuristicFunctions import EculdianDistance

# Define your breadth-first search function here
def breadth_first(grid, start, goal):

    # TODO: Replace the None values for 
        # "queue" and "visited" with data structure objects
        # and add the start position to each 
    q = Queue()
    q.put(start)
    visited = set()
    visited.add(start)
    branch = {}
    found = False
    
    # Run loop while queue is not empty
    while not q.empty(): # TODO: replace True with q.empty():
        # TODO: Replace "None" to remove the 
            #first element from the queue
        current_node = q.get()
        
        # TODO: Replace "False" to check if the current 
            # node corresponds to the goal state
        if current_node == goal: 
            print('Found a path.')
            found = True
            break
        else:
            # TODO: Get the new nodes connected to the current node
            # Iterate through each of the new nodes and:
            for a in valid_actions(grid, current_node):
                # delta of performing the action
                da = a.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # If the node has not been visited you will need to
                if next_node not in visited:
                    # 1. Mark it as visited                
                    visited.add(next_node)
                    # 2. Add it to the queue               
                    q.put(next_node)
                    # 3. Add how you got there to the branch dictionary 
                    branch[next_node] = (current_node, a)
                    
    # Now, if you found a path, retrace your steps through 
    # the branch dictionary to find out how you got there!

    path = []

    if found:
        path = retrivePath(start, goal, branch)
    return path

# Define your breadth-first search function here
def breadth_first_cost(grid, start, goal):

    # TODO: Replace the None values for 
        # "queue" and "visited" with data structure objects
        # and add the start position to each 
    q = PriorityQueue()
    q.put((0,start))
    visited = set()
    visited.add(start)
    branch = {}
    found = False
    
    # Run loop while queue is not empty
    while not q.empty(): # TODO: replace True with q.empty():
        # TODO: Replace "None" to remove the 
            #first element from the queue
        item = q.get()
        current_node = item[1]
        current_cost = item[0]
        
        # TODO: Replace "False" to check if the current 
            # node corresponds to the goal state
        if current_node == goal: 
            print('Found a path.')
            found = True
            break
        else:
            # TODO: Get the new nodes connected to the current node
            # Iterate through each of the new nodes and:
            for a in valid_actions(grid, current_node, ActionCost):
                # delta of performing the action
                da = a.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                new_cost = current_cost + da[2]
                # If the node has not been visited you will need to
                if next_node not in visited:
                    # 1. Mark it as visited                
                    visited.add(next_node)
                    # 2. Add it to the queue               
                    q.put((new_cost, next_node))
                    # 3. Add how you got there to the branch dictionary 
                    branch[next_node] = (new_cost, current_node, a)
                    
    # Now, if you found a path, retrace your steps through 
    # the branch dictionary to find out how you got there!

    path = []
    path_cost = 0

    if found:
        path,path_cost = retrivePathWithCost(start, goal, branch)
    return path,path_cost

def a_star_graph(graph, start, goal, h,Ac = ActionCost, valid_action = valid_actions):
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
    if(found):
        print("Path Found")
    else:
        print("path Not Found")
        branch =0

    return branch

def a_star(grid, start, goal, h,Ac = ActionCost, valid_action = valid_actions):
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for a in valid_action(grid, current_node, Ac):
                # get the tuple representation
                da = a.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: calculate branch cost (action.cost + g)
                branch_cost = current_cost + a.cost
                # TODO: calculate queue cost (action.cost + g + h)
                queue_cost = branch_cost + h(next_node,goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, a)
                    queue.put((queue_cost, next_node))
    if(found):
        print("Path Found")
    else:
        print("path Not Found")
        branch =0

    return branch

def a_star_graph_complete(graph, start, goal, h,Ac = ActionCost, valid_action = valid_actions):
    """Modified A* to work with NetworkX graphs."""
    
    # TODO: complete
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
    if(found):
        print("Path Found")
    else:
        print("path Not Found")
        branch =0

    return branch