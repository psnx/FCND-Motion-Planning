from enum import Enum
from queue import PriorityQueue
from selectors import EpollSelector
from typing import List, Tuple
from xmlrpc.client import Boolean
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
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
    NORTHEAST = (-1, 1, 1.41421356237)
    SOUTHEAST = (1, 1, 1.41421356237)
    SOUTHWEST = (1,-1, 1.41421356237)
    NORTHWEST = (-1,-1, 1.41421356237)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
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
        if Action.NORTHWEST in valid_actions: valid_actions.remove(Action.NORTHWEST)
        if Action.NORTHEAST in valid_actions: valid_actions.remove(Action.NORTHEAST)

    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
        if Action.SOUTHEAST in valid_actions: valid_actions.remove(Action.SOUTHEAST)
        if Action.SOUTHWEST in valid_actions: valid_actions.remove(Action.SOUTHWEST)

    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
        if Action.NORTHWEST in valid_actions: valid_actions.remove(Action.NORTHWEST)
        if Action.SOUTHWEST in valid_actions: valid_actions.remove(Action.SOUTHWEST)

        
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
        if Action.NORTHEAST in valid_actions: valid_actions.remove(Action.NORTHEAST)
        if Action.SOUTHEAST in valid_actions: valid_actions.remove(Action.SOUTHEAST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
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
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def Point(p):
    return np.array([p[0], p[1], 1.])

def collinearity_float(p1, p2, p3, epsilon=1e-6) -> bool: 
    mat = np.vstack((Point(p1), Point(p2), Point(p3)))
    det = np.linalg.det(mat)
    return abs(det) < epsilon

def collinearity_check(path: List[Tuple[int, int]], epsilon: float):
    yield False # The first point shall not be purged
    for i in range(len(path)-2):
        p1 = path[i]
        p2 = path[i+1]
        p3 = path[i+2]
        print("points", p1, p2, p3)
        print("coll:", collinearity_float(p1, p2, p3, epsilon))
        yield collinearity_float(p1, p2, p3, epsilon)
    yield False # the last point should not be purged

def prune(path: List[Tuple[int, int]]):
    new_path = [ p for p, c in zip(path, collinearity_check(path, epsilon=1e-10)) if not c ]
    print("after prune new path: ", new_path)
    return new_path

def set_valid_goal_around_target(grid, pos0: int, pos1: int, r = 20):
    print("grid at pos", grid[pos0, pos1])
    a, b = pos0, pos1
    while grid[a, b] > 0.0:
        a = np.random.randint(pos0 - r, pos0 + r)
        b = np.random.randint(pos1 - r, pos1 + r)
    print("goal ", a, b)
    return (a, b)
