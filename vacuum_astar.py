# Natalia Rumbuc
# Package needed: import heapq

# General Structure of the A* Search Algorithm 

# A* is a search algorithm that combines the cost from the start node (g(n)) 
# and an estimated cost to reach the goal (h(n)) to choose which nodes to expand. 
# The total cost function f(n)=g(n)+h(n).

# Initialize a priority queue (open list) with the initial state.
# Keep a closed list (visited nodes) to avoid revisiting the same state.
# Expand the node with the lowest f(n) value.
# For each neighbor (possible action), calculate f(n) and update the path if a better path is found.
# Stop when the goal state (all squares clean) is reached.
# Record the optimal path and the values of f(n) for each step.



import heapq

class Node:
    def __init__(self, state, parent=None, action=None, g=0, h=0):
        self.state = state # The grid and agent's position
        self.parent = parent # Parent node -> for backtracking the path
        self.action = action # action taken to reach node
        self.g = g # cost from the start node to this node
        self.h = h # heuristic cost from node to the goal
        self.f = g + h # total cost 
        
    def __lt__(self, other):
        return self.f < other.f
      
# The 5x5 grid environment 
class Grid:
    def __init__(self, dirty_squares, agent_position):
        self.grid_size = 5  # 5x5 grid
        self.dirty_squares = dirty_squares  # list of dirt squares
        self.agent_position = agent_position  # agent position

    def goal(self):
        return len(self.dirty_squares) == 0  # The goal is to have no dirty squares left

    def get_neighbors(self, node):
        # This method return possible actions and resulting states
        neighbors = []
        for action in self.actions(node.state):
            new_state = self.result(node.state, action)
            neighbors.append((action, new_state))
        return neighbors

    def actions(self, state):
        # Returns the possible actions the agent can perform in a given state.
        actions = ['left', 'right', 'up', 'down', 'suck']
        x, y = state['agent_position']

        # If the vacuum is on the left edge of 5x5 grid, remove action move left.
        if x == 0:
            actions.remove('left')
        # If the vacuum is on the right edge of 5x5 grid, remove action move right.
        if x == self.grid_size - 1:
            actions.remove('right')
        # If the vacuum is on the bottom edge of 5x5 grid, remove action move down.
        if y == 0:
            actions.remove('down')
        # If the vacuum is on the top edge of 5x5 grid, remove action move up.
        if y == self.grid_size - 1:
            actions.remove('up')
        # If the square is not dirty, remove action suck.
        if (x, y) not in state['dirty_squares']:
            actions.remove('suck')

        return actions

    def result(self, state, action):
        # Return the state that results from executing an action in the original state.
        new_state = {
            'agent_position': list(state['agent_position']),
            'dirty_squares': state['dirty_squares'][:]  # Use a copy of the dirty squares
        }
        x, y = state['agent_position']

        if action == 'left':
            new_state['agent_position'][0] -= 1
        elif action == 'right':
            new_state['agent_position'][0] += 1
        elif action == 'down':
            new_state['agent_position'][1] -= 1
        elif action == 'up':
            new_state['agent_position'][1] += 1
        elif action == 'suck':
            if (x, y) in new_state['dirty_squares']:
                new_state['dirty_squares'].remove((x, y))

        return new_state

    # Heuristic functions 
def h1(state):
    agent1, agent2 = state['agent_position']
    dirty_squares = state['dirty_squares']
    
    # If there are no dirt squares left, then the heuristic is equal to 0
    if len(dirty_squares) == 0:  
        return 0 
    
    # Manhattan distance to the nearest dirty square
    nearest_dirty_square = min(abs(agent1 - x) + abs(agent2 - y) for (x, y) in dirty_squares)
    
    # Total: nearest dirty distance + number of dirty squares left
    return nearest_dirty_square + len(dirty_squares)

def h2(state):
    agent1, agent2 = state['agent_position']
    dirty_squares = state['dirty_squares']
    
    # If there are no dirt squares left, then the heuristic is equal to 0
    if len(dirty_squares) == 0:  
        return 0 
    
    # Manhattan distance to the farthest dirty square
    farthest_dirty_square = max(abs(agent1 - x) + abs(agent2 - y) for (x, y) in dirty_squares)
    
    # Total: farthest dirty distance + number of dirty squares left
    return farthest_dirty_square + len(dirty_squares)

# A* Algorithm
def astar(grid, heuristic):
    open_list = []
    closed_list = set()
    
    initial_state = {
        'agent_position': grid.agent_position,
        'dirty_squares': grid.dirty_squares
    }
    
    start_node = Node(initial_state, g=0, h=heuristic(initial_state))
    
    # Push initial node into priority queue
    heapq.heappush(open_list, start_node)
    
    expanded_nodes = 0
    
    while open_list:
        # Pop the node with the lowest f value
        current_node = heapq.heappop(open_list)
        
        # Check if this node's state is the goal
        if len(current_node.state['dirty_squares']) == 0:  # Check the goal condition directly
            return reconstruct_path(current_node), expanded_nodes
        
        # Tuple for the agent position and a frozenset for dirty squares to ensure they are hashable
        closed_list.add((tuple(current_node.state['agent_position']), frozenset(current_node.state['dirty_squares'])))
        
        # Expand the node
        for action, new_state in grid.get_neighbors(current_node):
            new_node = Node(new_state, parent=current_node, action=action, g=current_node.g + 1, h=heuristic(new_state))
            
            # Check if the new node's state is in the closed list
            if (tuple(new_node.state['agent_position']), frozenset(new_node.state['dirty_squares'])) in closed_list:
                continue
            
            heapq.heappush(open_list, new_node)
            expanded_nodes += 1
    
    return None, expanded_nodes 

# Reconstruct the path from the goal state
def reconstruct_path(node):
    actions = []
    f_values = []
    while node is not None:
        actions.append(node.action)
        f_values.append(node.f)
        node = node.parent
    return actions[::-1], f_values[::-1]

# Part A
def part_a():
    dirty_squares = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]  # Top 5 squares are dirty
    agent_position = (1, 1)  # Agent starts in the lower-left corner

    grid = Grid(dirty_squares, agent_position)
    actions, f_values = astar(grid, h1)

    print("Part A:")
    print("Actions:", actions)
    print("f(n) values:", f_values)
    if isinstance(f_values, list):
        print("Nodes expanded:", len(f_values))

# Part B
def part_b():
    dirty_squares = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]  # Top 5 squares are dirty
    agent_position = (1, 1)  # Agent starts in the lower-left corner

    grid = Grid(dirty_squares, agent_position)
    actions, f_values = astar(grid, h2)

    print("Part B:")
    print("Actions:", actions)
    print("f(n) values:", f_values)
    if isinstance(f_values, list):
        print("Nodes expanded:", len(f_values))

# Run both parts
part_a()  # Run Part A with h1
part_b()  # Run Part B with h2     