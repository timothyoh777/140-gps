import graphgui as maps


class Queue:
    """
    A simple implementation of a FIFO queue.
    """
    def __init__(self):
        """
        Initializes the queue.
        """
        self._queue = []
        
    def __len__(self):
        """
        Returns: an integer representing the number of items in the queue.
        """
        return len(self._queue)
    
    def __str__(self):
        """
        Returns: a string representation of the queue.
        """
        return str(self._queue)
    
    def push(self, item):
        """
        Adds the input item to the queue.

        Input:
            - item: any data type that's valid in a list
        """
        self._queue.append(item)
        
    def pop(self):
        """
        Removes the least recently added item from the queue. 

        Assumes that there is at least one element in the queue. It
        is an error if the queue is empty. You do not need to check for
        this condition.

        Returns: the least recently added item
        """
        return self._queue.pop(0)
        
    def clear(self):
        """
        Removes all items from the queue.
        """
        self._queue = []

class Stack:
    """
    A simple implementation of a LIFO stack.
    """
    def __init__(self):
        """
        Initializes the stack.
        """
        self._stack = []
        
    def __len__(self):
        """
        Returns: an integer representing the number of items in the stack.
        """
        return len(self._stack)
    
    def __str__(self):
        """
        Returns: a string representation of the stack.
        """
        return str(self._stack)
    
    def push(self, item):
        """
        Adds the input item to the stack.

        Input:
            - item: any data type that's valid in a list
        """
        self._stack.append(item)
        
    def pop(self):
        """
        Removes the least recently added item from the stack. 

        Assumes that there is at least one element in the stack. It
        is an error if the stack is empty. You do not need to check for
        this condition.

        Returns: the least recently added item
        """
        return self._stack.pop()
        
    def clear(self):
        """
        Removes all items from the stack
        """
        self._stack = []


def bfs_dfs(graph, rac_class, start_node, end_node):
    """
    Performs a breadth-first search or a depth-first search on graph starting
    at the start_node. The rac_class should either be a reference to the
    Queue class (the class itself, NOT an instance of it) or a reference to 
    the Stack class to select BFS or DFS, respectively.

    Terminates when end_node is found or the entire graph has been searched.

    Inputs:
        - graph:      a directed Graph object representing a street map
        - rac_class:  a reference to a restricted access container class (Queue 
                      or Stack) class to use for the search
        - start_node: a node in graph representing the start
        - end_node:   a node in graph representing the end

    Returns: a dictionary associating each visited node with its parent node
    """
    #creates a restricted access class object depending on function input 
    rac = rac_class()
    #create dictionaries to store node distance pairs and node parent pairs
    dist = {}
    parent_dic = {}
    
    #set all node's distances to infinity, which means they have not been explored yet
    for node in graph.nodes():
        dist[node] = float("inf")
        parent_dic[node] = None
    
    #push start node into restricted access container set distance of start node to 0
    dist[start_node] = 0
    rac.push(start_node)
    
    while len(rac) > 0:
        #pop node off of restriced access container
        current_node = rac.pop()
        #explore neighbors of popped off node
        for nbr in graph.get_neighbors(current_node):
            if dist[nbr] == float("inf"):
                dist[nbr] = dist[current_node] + 1
                parent_dic[nbr] = current_node
                rac.push(nbr)
            if nbr == end_node:
                return parent_dic
               
    return parent_dic
    
    
 
def recursive_dfs(graph, start_node, end_node, parent):
    """
    Performs a recursive depth-first search on graph starting at the
    start_node, updating the parent mapping as it goes.

    Terminates when end_node is found or entire graph has been searched.

    Inputs:
        - graph:      a directed Graph object representing a street map
        - start_node: a node in graph representing the start
        - end_node:   a node in graph representing the end
        - parent:     a dictionary that initially has one entry associating
                      the original start_node with None
    """
    
    
    
    #for neighbors of start node
    for nbr in graph.get_neighbors(start_node):
        #if end node is explored return parent
        if end_node in parent:
            return parent
        #if neighbor is not explored, call dfs on neighbor
        if nbr not in parent:
            parent[nbr] = start_node
            parent = recursive_dfs(graph, nbr, end_node, parent)
    return parent



def astar(graph, start_node, end_node, edge_distance, heuristic_distance):
    """
    Performs an A* search on graph starting at start_node.

    Terminates when end_node is found or entire graph has been searched.

    Inputs:
        - graph:              a directed Graph object representing a street map
        - start_node:         a node in graph representing the start of the search
        - end_node:           a node in graph representing the end of the search
        - edge_distance:      a function which takes two neighboring nodes and
                              a graph and returns the edge distance between those
                              two nodes
        - heuristic_distance: a function which takes two nodes and a graph and 
                              returns the straight line distance between those 
                              two nodes

    Returns: a dictionary associating each visited node with its parent node
    """
    #initialize sets and containers
    open_set = []
    closed_set = []
    parent = {}
    open_set.append(start_node)
    parent[start_node] = None
    g_cost = {}
    h_cost = {}
    f_cost = {}
    #initilize values of start_node
    g_cost[start_node] = 0
    h_cost[start_node] = heuristic_distance(start_node, end_node, graph)
    f_cost[start_node] = g_cost[start_node] + h_cost[start_node]
    #loop until open set is empty
    while len(open_set) > 0:
        min_node = open_set[0]
        min_cost = g_cost[min_node] + f_cost[min_node]
        # pop node with smallest f cost
        for node in open_set:
            if g_cost[node] + heuristic_distance(node, end_node, graph) < min_cost:
                min_cost =g_cost[node] + heuristic_distance(node, end_node, graph)
                min_node = node
        open_set.remove(min_node)
        closed_set.append(min_node)
        #find parent of node by checking neighbors
        for nbr in graph.get_neighbors(min_node):
            if nbr in closed_set:
                continue
            if nbr not in closed_set and nbr not in open_set:
                open_set.append(nbr)
                g_cost[nbr] = g_cost[min_node] + edge_distance(min_node, nbr, graph)
                h_cost[nbr] = heuristic_distance(nbr, end_node, graph)
                f_cost[nbr] = g_cost[nbr] + h_cost[nbr]
                parent[nbr] = min_node
            
            if nbr in open_set:
                if edge_distance(min_node, nbr, graph) + g_cost[min_node] < g_cost[nbr]:
                    g_cost[nbr] = edge_distance(min_node, nbr, graph) + g_cost[min_node]
                    h_cost[nbr] = heuristic_distance(nbr, end_node, graph)
                    f_cost[nbr] = g_cost[nbr] + h_cost[nbr]
                    parent[nbr] = min_node
    return parent



maps.start(bfs_dfs, Queue, Stack, recursive_dfs, astar)
