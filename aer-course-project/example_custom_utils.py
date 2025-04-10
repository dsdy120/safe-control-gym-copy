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

def map_generation(res):
    NN = round(7/res)
    M = np.zeros((NN, NN), dtype=int)

    obs = np.array([[1.5, -2.5], [0.5,-1], [1.5,0], [-1,0]])

    for _, coord in enumerate(obs):
        M[round((coord[0]-0.26+3.5)/res):round((coord[0]+0.26+3.5)/res), round((coord[1]-0.26+3.5)/res):round((coord[1]+0.26+3.5)/res)] = 1
    
    #gate_vertical = np.array([[0.5, -2.275], [0.5, -2.725], [0,0.425], [0,-0.025]])
    #gate_horizontal = np.array([[2.225,-1.5], [1.775,-1.5], [-0.725,1.5], [-0.275,1.5]])
    #t = 0.3
    #opening = 0.15
    #offset = (opening/2) + (t/2)
    #length = 0.3

    #gate_vertical = np.array([[0.5, -2.5-offset], [0.5, -2.5+offset], [0,0.2-offset], [0,0.2+offset]])    
    #gate_horizontal = np.array([[2.0-offset,-1.5], [2.0+offset,-1.5], [-0.5-offset,1.5], [-0.5+offset,1.5]])
    gate_vertical = np.array([[0.5, -2.5], [0,0.2]])   
    gate_horizontal = np.array([[2.0,-1.5], [-0.5,1.5]])
    num1 = 5
     

    for _, coord in enumerate(gate_vertical):
        #y = round((coord[1]-0.025+3.5)/res)
        #M[round((coord[0]-0.06+3.5)/res):round((coord[0]+0.06+3.5)/res), y:(y+1)] = 1
        #M[round((coord[0]-(length/2)+3.5)/res):round((coord[0]+(length/2)+3.5)/res), round((coord[1]-(t/2)+3.5)/res):round((coord[1]+(t/2)+3.5)/res)] = 1
        x = round((coord[0]+3.5)/res)
        y = round((coord[1]+3.5)/res)
        M[(x-1):(x+2), (y-3):y] = 1
        M[(x-1):(x+2), (y+1):(y+4)] = 1

    for _, coord in enumerate(gate_horizontal):
        #x = round((coord[0]-0.025+3.5)/res)
        #M[x:(x+1), round((coord[1]-0.06+3.5)/res):round((coord[1]+0.06+3.5)/res)] = 1
        #M[round((coord[0]-(t/2)+3.5)/res):round((coord[0]+(t/2)+3.5)/res), round((coord[1]-(length/2)+3.5)/res):round((coord[1]+(length/2)+3.5)/res)] = 1
        x = round((coord[0]+3.5)/res)
        y = round((coord[1]+3.5)/res)
        M[(x-3):x, (y-1):(y+2)] = 1
        M[(x+1):(x+4), (y-1):(y+2)] = 1
        

    return M


def update_map (M, path, gate_num, gate_coord, res):
    if gate_num == 1 or gate_num == 3:
        x = gate_coord[0] + (path[-2][0]-gate_coord[0])
        M[x:(x+1), round(gate_coord[1]-(0.2/res)):round(gate_coord[1]+(0.2/res))] = 1

    else:
        y = gate_coord[1] + (path[-2][1]-gate_coord[1])
        M[round(gate_coord[0] - (0.2/res)):round(gate_coord[0] + (0.2/res)), y:(y+1)] = 1

    return M


#def plot_map(M, res, path1, path2, path3, path4, path5):
def plot_map(M, res, path1):
    plt.figure(figsize=(8, 8))
    NN = round(7/res)
    for i in range(NN):
        for j in range(NN):
            if M[i, j] == 1:
                plt.plot(i, j, 'rx')
            #else:
                #plt.plot(i, j, 'kx')
                
    plt.plot((-1+3.5)/res, (-3+3.5)/res, 'go', label='Start')
    plt.plot((-0.5+3.5)/res, (2+3.5)/res, 'bo', label='End')
    plt.plot(path1[:, 0], path1[:, 1], 'b-', linewidth=2, label='Path')
    """plt.plot(path2[:, 0], path2[:, 1], 'g-', linewidth=2, label='Path')
    plt.plot(path3[:, 0], path3[:, 1], 'k-', linewidth=2, label='Path')
    plt.plot(path4[:, 0], path4[:, 1], 'y-', linewidth=2, label='Path')
    plt.plot(path5[:, 0], path5[:, 1], 'c-', linewidth=2, label='Path')"""
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title("A* Pathfinding Result")
    plt.show()

class path_planning():
    def __init__(self, res, gate_order, M):
        self.res = res
        self.NN = round(7/res)
        self.M = map_generation(res)
        
        self.gate_coord = [
            np.array([round((0.5+3.5)/res), round((-2.5+3.5)/res)]),   # gate_1
            np.array([round((2.0+3.5)/res), round((-1.5+3.5)/res)]),   # gate_2
            np.array([round((0+3.5)/res), round((0.2+3.5)/res)]),      # gate_3
            np.array([round((-0.5+3.5)/res), round((1.5+3.5)/res)])    # gate_4
        ]

        self.gate_order = gate_order

        self.gate_1 = self.gate_coord[gate_order[0] - 1]
        self.gate_2 = self.gate_coord[gate_order[1] - 1]
        self.gate_3 = self.gate_coord[gate_order[2] - 1]
        self.gate_4 = self.gate_coord[gate_order[3] - 1]

        self.start = np.array([int((-1+3.5)/res), int((-3+3.5)/res)])
        self.end = np.array([int((-0.5+3.5)/res), int((2+3.5)/res)])

    def run_Astar(self):
        path_segments = []

        for i, (start, end, gate_num, gate_coord) in enumerate([
            (self.start, self.gate_1, self.gate_order[0], self.gate_1),
            (self.gate_1, self.gate_2, self.gate_order[1], self.gate_2),
            (self.gate_2, self.gate_3, self.gate_order[2], self.gate_3),
            (self.gate_3, self.gate_4, self.gate_order[3], self.gate_4),
            (self.gate_4, self.end, None, None)
        ]):
            path = self.A_star(start, end, self.M)
            if path is None or len(path) == 0:
                print(f"Path segment {i+1} failed. Aborting.")
                return None
            if i < 4:  # Only update map for gate segments
                self.M = update_map(self.M, path, gate_num, gate_coord, self.res)

            #path = self.downsample_path(path)
            path_segments.append(path)

        full_path = np.vstack([seg if i == 0 else seg[1:] for i, seg in enumerate(path_segments)])  # avoid duplicates
        return full_path
    

    """def run_Astar(self):
        path_1 = self.A_star(self.start, self.gate_1, self.M)
        self.M = update_map(self.M, path_1, gate_order[0], self.gate_1, res)
        path_2 = self.A_star(self.gate_1, self.gate_2, self.M)
        self.M = update_map(self.M, path_2, gate_order[1], self.gate_2, res)
        path_3 = self.A_star(self.gate_2, self.gate_3, self.M)
        self.M = update_map(self.M, path_3, gate_order[2], self.gate_3, res)
        path_4 = self.A_star(self.gate_3, self.gate_4, self.M)
        self.M = update_map(self.M, path_4, gate_order[3], self.gate_4, res)
        path_5 = self.A_star(self.gate_4, self.end, self.M)
        path = np.vstack([path_1, path_2, path_3, path_4, path_5])
        path_smooth = self.downsample_path(path)
        
        #return path_1, path_2, path_3, path_4, path_5
        return path_smooth"""
        
    def A_star(self, path_start, path_end, M):
        
        enable_diagonal = True

        h0 = self.heuristic(*path_start, *path_end, enable_diagonal)
        openlist = [[path_start[0], path_start[1], 0, h0, h0, -1, -1]]  # x, y, g, h, f, parent_x, parent_y
        closelist = []

        while True:
            current_index = self.lowest_f_index(openlist)
            
            if current_index == -1:
                print("No solution found!")
                return

            current = openlist.pop(current_index)
            closelist.append(current)

            if self.is_same_location(current[0], current[1], *path_end):
                print("Solution found!")
                path = self.reconstruct_path(closelist)
                break

            g_list, neighbors = self.cost_neighbors(M, current[2], self.NN, self.NN, current[0], current[1], closelist, enable_diagonal)
            for g, (nx, ny) in zip(g_list, neighbors):
                h = self.heuristic(nx, ny, *self.end, enable_diagonal)
                f = g + h
                in_open, idx = self.is_in_list(openlist, nx, ny)
                if in_open:
                    if g < openlist[idx][2]:  # update if better g
                        openlist[idx][2] = g
                        openlist[idx][4] = f
                        openlist[idx][5:7] = [current[0], current[1]]
                else:
                    openlist.append([nx, ny, g, h, f, current[0], current[1]])
        return path
    
    def heuristic(self, x1, y1, x2, y2, diagonal=True):
        if diagonal:
            return np.hypot(x1 - x2, y1 - y2)
        else:
            return abs(x1 - x2) + abs(y1 - y2)

    def is_same_location(self, x1, y1, x2, y2):
        return int(x1) == int(x2) and int(y1) == int(y2)

    def is_in_list(self, lst, x, y):
        for i, item in enumerate(lst):
            if item[0] == x and item[1] == y:
                return True, i
        return False, -1

    def cost_neighbors(self, M, g_parent, size_x, size_y, x, y, closelist, diagonal=True):
        directions = [(1,0), (-1,0), (0,1), (0,-1)]
        if diagonal:
            directions += [(1,1), (1,-1), (-1,1), (-1,-1)]
        
        g_list = []
        index_list = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < size_x and 0 <= ny < size_y and M[nx, ny] == 0:
                if not self.is_in_list(closelist, nx, ny)[0]:
                    g = np.hypot(dx, dy) + g_parent
                    g_list.append(g)
                    index_list.append([nx, ny])
        return g_list, index_list

    def lowest_f_index(self, openlist):
        if not openlist:
            return -1
        return np.argmin([node[4] for node in openlist])

    def reconstruct_path(self, closelist):
        path = [[closelist[-1][0], closelist[-1][1]]]
        parent = [closelist[-1][5], closelist[-1][6]]
        while not self.is_same_location(parent[0], parent[1], -1, -1):
            path.insert(0, parent[:])
            _, loc = self.is_in_list(closelist, parent[0], parent[1])
            parent = [closelist[loc][5], closelist[loc][6]]
        return np.array(path)
    


    def downsample_path(self, path_meters):
        """
        Smooth and downsample the (x, y) path using B-spline interpolation.
        Append constant altitude (z).
        
        Args:
            path_meters: Nx2 array of (x, y) in meters
            num_points: number of interpolated waypoints
            altitude: constant altitude for each point
        
        Returns:
            waypoints: Nx3 array of (x, y, z)
        """
        num_points = 20
        altitude = 1
        # Extract x and y
        x = path_meters[:, 0]
        y = path_meters[:, 1]
        
        # Parameterize path by cumulative distance
        distance = np.cumsum(np.sqrt(np.diff(x, prepend=x[0])**2 + np.diff(y, prepend=y[0])**2))
        distance = distance / distance[-1]  # Normalize to [0, 1]

        # Fit B-spline
        tck, _ = interpolate.splprep([x, y], s=0)
        
        # Resample with fewer points
        u_fine = np.linspace(0, 1, num_points)
        x_smooth, y_smooth = interpolate.splev(u_fine, tck)

        # Append constant z
        z = np.ones_like(x_smooth) * altitude
        waypoints = np.vstack([x_smooth, y_smooth, z]).T
        #waypoints = np.vstack([x_smooth, y_smooth]).T

        return waypoints

