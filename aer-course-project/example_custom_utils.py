"""Example utility module.

Please use a file like this one to add extra functions.

"""

def exampleFunction():
    """Example of user-defined function.

    """
    x = -1
    return x

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

# generate map with obstacles (1) and free space (0)
def map_generation(res, obs):
    
    NN = round(7/res) # number of nodes
    M = np.zeros((NN, NN), dtype=int) # initialize map with zeros (free)

    # set obstacles if true
    if obs == 1:
        obs = np.array([[1.5, -2.5], [0.5,-1], [1.5,0], [-1,0]])
        for _, coord in enumerate(obs):
            M[round((coord[0]-0.26+3.5)/res):round((coord[0]+0.26+3.5)/res), round((coord[1]-0.26+3.5)/res):round((coord[1]+0.26+3.5)/res)] = 1
    
    # set gates
    gate_vertical = np.array([[0.5, -2.5], [0,0.2]])   
    gate_horizontal = np.array([[2.0,-1.5], [-0.5,1.5]]) 

    t = 3 # thickness of the gate

    # create obstacles around gates (determined experimentally)
    for i, coord in enumerate(gate_vertical):
        x = round((coord[0]+3.5)/res)
        y = round((coord[1]+3.5)/res)

        if i == 0:
            M[(x-5):(x+6), (y-1)] = 1
            M[(x-5):(x+6), (y+1)] = 1

            M[(x-2):(x+3), (y-t):y] = 1
            M[(x-2):(x+3), (y+1):(y+t+1)] = 1

        else:
            M[(x-4):(x+6), (y-1)] = 1
            M[(x-4):(x+6), (y+1)] = 1

            M[(x-2):(x+3), (y-t):y] = 1
            M[(x-2):(x+3), (y+1):(y+t+1)] = 1


    for i, coord in enumerate(gate_horizontal):
        x = round((coord[0]+3.5)/res)
        y = round((coord[1]+3.5)/res)

        if i == 0:
            M[(x+1), (y-5):(y+6)] = 1
            M[(x-1), (y-5):(y+6)] = 1

            M[(x-t):x, (y-2):(y+3)] = 1
            M[(x+1):(x+t+1), (y-2):(y+3)] = 1

        else:
            M[(x+1), (y-5):(y+2)] = 1
            M[(x-1), (y-5):(y+2)] = 1

            M[(x-t):x, (y-2):(y+3)] = 1
            M[(x+1):(x+t+1), (y-2):(y+3)] = 1

    return M

# update map by closing the gate on the side the drone entered
def update_map(M, path, gate_num, gate_coord):
    
    if gate_num == 1 or gate_num == 3:
        x = gate_coord[0] + (path[-2][0]-gate_coord[0])
        M[x:(x+1), gate_coord[1]] = 1

    else:
        y = gate_coord[1] + (path[-2][1]-gate_coord[1])
        M[gate_coord[0], y:(y+1)] = 1

    return M


# plot the map and the path
def plot_map(M, res, path1):

    plt.figure(figsize=(8, 8))
    NN = round(7/res)

    # plot obstacles as black x
    for i in range(NN):
        for j in range(NN):
            if M[i, j] == 1:
                plt.plot(i, j, 'kx')
                
    plt.plot(path1[:, 0], path1[:, 1], 'b-', linewidth=2, label='Path')
    plt.plot((-1+3.5)/res, (-3+3.5)/res, 'go', label='Start')
    plt.plot((-0.5+3.5)/res, (2+3.5)/res, 'ro', label='End')
    
    x = np.array([(0.5+3.5)/res, (2.0+3.5)/res, 3.5/res, (-0.5+3.5)/res])
    y = np.array([(-2.5+3.5)/res, (-1.5+3.5)/res, (0.2+3.5)/res, (1.5+3.5)/res])
    plt.plot(x, y, 'r*', label='Gates')

    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title("A* Pathfinding Result")
    plt.show()

class path_planning():
    def __init__(self, res, gate_order, obs):
        
        # set parameters
        self.res = res
        self.NN = round(7/res)
        self.obs = obs
        self.gate_order = gate_order
        
        # calculate coordinates of gates
        self.gate_coord = [
            np.array([round((0.5+3.5)/res), round((-2.5+3.5)/res)]),   # gate_1
            np.array([round((2.0+3.5)/res), round((-1.5+3.5)/res)]),   # gate_2
            np.array([round((0+3.5)/res), round((0.2+3.5)/res)]),      # gate_3
            np.array([round((-0.5+3.5)/res), round((1.5+3.5)/res)])    # gate_4
        ]
        print(self.gate_coord)

        # set start and end coordinates
        self.start = np.array([int((-1+3.5)/res), int((-3+3.5)/res)])
        self.end = np.array([int((-0.5+3.5)/res), int((2+3.5)/res)])

    # run A* algorithm for each segment
    def run_Astar(self):
        
        path_segments = [] # initialize

        # run A* for each segment
        for i in range(len(self.gate_order)+1):

            M = map_generation(self.res, self.obs) # create the original map
            
            # start point to gate 1
            if i == 0:
                start = self.start # set start of path segment
                end = self.gate_coord[self.gate_order[i] - 1] # set end of path segment
                M_updated = M # initialize

            # last gate to end point
            elif i == len(self.gate_order):
                start = self.gate_coord[self.gate_order[i-1] - 1] # set start of path segment
                end = self.end # set end of path segment

            # gate to gate
            else:
                start = self.gate_coord[self.gate_order[i-1] - 1] # set start of path segment
                end = self.gate_coord[self.gate_order[i] - 1] # set end of path segment

            # run A* algorithm to find optimal path segment
            path = self.A_star(start, end, M_updated)

            # close one side of gate so that drone passes through the gate
            if i!=len(self.gate_order):
                M_updated = update_map(M, path, self.gate_order[i], self.gate_coord[self.gate_order[i] - 1]) # update map with gate closed

            #path = self.sample_path(path) # smooth the path
            
            x = path[:, 0] # get x
            y = path[:, 1] # get y
            altitude = 1 # append constant z of 1m
            z = np.ones_like(x) * (altitude+3.5)/self.res # get z           
            path = np.vstack([x, y, z]).T # stack x, y, z
            
            path_segments.append(path) # store the entire path

        full_path = np.vstack([seg if i == 0 else seg[1:] for i, seg in enumerate(path_segments)])  # avoid duplicates
        
        return full_path
        
    def A_star(self, path_start, path_end, M):
    
        enable_diagonal = True # enable diagonal movement

        h0 = self.heuristic(*path_start, *path_end, enable_diagonal) # set initial heuristic
        openlist = [[path_start[0], path_start[1], 0, h0, h0, -1, -1]]  # x, y, g, h, f, parent_x, parent_y
        closelist = []

        while True:

            # get the node index with the lowest f value
            current_index = self.lowest_f_index(openlist) 
            
            # if openlist is empty, no solution found
            if current_index == -1:
                print("No solution found!")
                return

            # remove the node with the lowest f value from openlist and add it to closelist
            current = openlist.pop(current_index)
            closelist.append(current)

            # if the current node is the goal, reconstruct the path
            if self.is_same_location(current[0], current[1], *path_end):
                print("Solution found!")
                path = self.reconstruct_path(closelist)
                break

            # get the neighbors of the current node and their costs
            g_list, neighbors = self.cost_neighbors(M, current[2], self.NN, self.NN, current[0], current[1], closelist, enable_diagonal)
            for g, (nx, ny) in zip(g_list, neighbors):
                h = self.heuristic(nx, ny, *self.end, enable_diagonal) # calculate heuristic    
                f = g + h # calculate total cost
                in_open, idx = self.is_in_list(openlist, nx, ny) # check if neighbor is in openlist
                if in_open:
                    if g < openlist[idx][2]:  # update if better cost to come
                        openlist[idx][2] = g
                        openlist[idx][4] = f
                        openlist[idx][5:7] = [current[0], current[1]]
                else:
                    openlist.append([nx, ny, g, h, f, current[0], current[1]])
        return path
    
    # calculate heuristic distance
    def heuristic(self, x1, y1, x2, y2, diagonal):
        if diagonal:
            # if diagonal movement is allowed, use Euclidean distance
            return np.hypot(x1 - x2, y1 - y2)
        else:
            # if diagonal is not allowedd, use Manhattan distance
            return abs(x1 - x2) + abs(y1 - y2)

    # check if two locations are the same
    def is_same_location(self, x1, y1, x2, y2):
        return int(x1) == int(x2) and int(y1) == int(y2)

    # check if a location is in a list
    def is_in_list(self, lst, x, y):
        # check if the location is in the list
        for i, item in enumerate(lst):
            if item[0] == x and item[1] == y:
                return True, i # return index if found
        return False, -1 # if not found, return -1

    # get the neighbors of a node and their costs
    def cost_neighbors(self, M, g_parent, size_x, size_y, x, y, closelist, diagonal):
        
        directions = [(1,0), (-1,0), (0,1), (0,-1)] # all allowed steps
        
        if diagonal:
            directions += [(1,1), (1,-1), (-1,1), (-1,-1)] # add diagonal steps if allowed
        
        # initialize
        g_list = []
        index_list = []
        
        # loop through all possible steps
        for dx, dy in directions:
            
            nx, ny = x + dx, y + dy # new point

            # check if the new point is within bounds and not an obstacle
            if 0 <= nx < size_x and 0 <= ny < size_y and M[nx, ny] == 0:
                # check if the new point is not in the closelist
                if not self.is_in_list(closelist, nx, ny)[0]:
                    g = np.hypot(dx, dy) + g_parent # calculate cost
                    g_list.append(g) # append cost to list
                    index_list.append([nx, ny]) # append new point to list

        return g_list, index_list

    # get the index of the node with the lowest f value (i.e., the next node to explore)
    def lowest_f_index(self, openlist):
        # if openlist is empty, return -1
        if not openlist:
            return -1
        # return the index of the node with the lowest f value
        return np.argmin([node[4] for node in openlist])

    # reconstruct the path from the closelist
    def reconstruct_path(self, closelist):
        path = [[closelist[-1][0], closelist[-1][1]]] # get the last node in the closelist
        parent = [closelist[-1][5], closelist[-1][6]] # get the parent of the last node

        # loop until the parent is the start node
        while not self.is_same_location(parent[0], parent[1], -1, -1):
            path.insert(0, parent[:]) # insert the parent node at the beginning of the path
            _, loc = self.is_in_list(closelist, parent[0], parent[1]) # get the index of the parent node
            parent = [closelist[loc][5], closelist[loc][6]] # set the new parent node
        
        return np.array(path)
    
    # smooth the path using B-spline interpolation
    def sample_path(self, path):

        # number of points to sample
        num_points = 500
        
        # get x and y
        x = path[:, 0]
        y = path[:, 1]
        
        # parameterize path by cumulative distance
        distance = np.cumsum(np.sqrt(np.diff(x, prepend=x[0])**2 + np.diff(y, prepend=y[0])**2))
        distance = distance / distance[-1]  # normalize to [0, 1]

        # fit B-spline
        tck, _ = interpolate.splprep([x, y], s=0)
        
        # resample
        u_fine = np.linspace(0, 1, num_points)
        x_smooth, y_smooth = interpolate.splev(u_fine, tck)

        # append constant z of 1m
        altitude = 1
        z = np.ones_like(x_smooth) * altitude
        
        # stack x, y, z
        waypoints = np.vstack([x_smooth, y_smooth, z]).T

        return waypoints