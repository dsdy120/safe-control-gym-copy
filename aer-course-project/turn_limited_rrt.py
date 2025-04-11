"""

RRT algorithm limited to connecting nodes that respect turn radius and speed limits.

"""
import abc
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
import matplotlib.animation as animation
import time

TIMESTEP_SEC = 1.0
MAX_ACCELERATION = 0.707
MAX_DEVIATION = 0.5 * MAX_ACCELERATION * TIMESTEP_SEC**2

AVG_TURN_RADIUS = 1.0
AVG_SPEED = np.sqrt(MAX_ACCELERATION * AVG_TURN_RADIUS)
STEP_LENGTH = AVG_SPEED * TIMESTEP_SEC

MIN_SAMPLING_DISTANCE = 0

print(f"Average speed is {AVG_SPEED} m/s")

class TreeNode:
    '''
    Class to represent a node in the RRT tree.
    '''
    def __init__(self, x, y, parent=None):
        self._x = x
        self._y = y
        self._parent = parent
        self._children = []
        if parent is not None:
            parent.add_child(self)
        
    def add_child(self, child):
        '''
        Add a child node to the current node.
        '''
        self._children.append(child)
    def add_parent(self, parent):
        '''
        Add a parent node to the current node. Automatically adds self as child for parent.
        '''
        if self._parent is not None:
            raise ValueError(f"Node already has a parent: {self.get_position()}")
        self._parent = parent
        parent.add_child(self)
    def get_children(self):
        '''
        Get the children of the current node.
        '''
        return self._children
    def get_parent(self):
        '''
        Get the parent of the current node.
        '''
        return self._parent
    def get_position(self):
        '''
        Get the position of the current node.
        '''
        return self._x, self._y
    def get_rel_position(self):
        '''
        Get the relative position of the current node with respect to the parent.
        '''
        if self._parent is None:
            return 0, 0
        else:
            parent_x, parent_y = self.get_parent().get_position()
            return self._x - parent_x, self._y - parent_y
    def get_fwd_position(self):
        '''
        Get the forward position of the current node with respect to the parent.
        '''
        if self._parent is None:
            return 0, 0
        else:
            rel_x, rel_y = self.get_rel_position()
            norm = np.sqrt(rel_x**2 + rel_y**2)
            return self._x + rel_x/norm*STEP_LENGTH, self._y + rel_y/norm*STEP_LENGTH
        
    def get_distance(self, xk, yk):
        '''
        Get the distance from the current node to a point (xk,yk).
        '''
        return np.sqrt((self._x - xk)**2 + (self._y - yk)**2)
    
    def get_fwd_deviation(self, xk, yk):
        '''
        Get the distance from a point (xk,yk) to the forward position of the current node.
        '''
        fwd_x, fwd_y = self.get_fwd_position()
        return np.sqrt((fwd_x - xk)**2 + (fwd_y - yk)**2)
    def get_path(self):
        '''
        Get the path from the root node to the current node.
        '''
        path = []
        node = self
        while node is not None:
            path.append(node)
            node = node.get_parent()
        return path[::-1]

class KeepOutBox:
    def __init__(self, x0,x1,y0,y1):
        self.x0 = x0
        self.x1 = x1
        self.y0 = y0
        self.y1 = y1
        pass

    def detect_collision(self, xi, yi, xf, yf):
        '''
        Check if a line segment from (xi,yi) to (xf,yf) intersects with the obstacle.
        '''
        # Check if the line segment intersects with the rectangle defined by the obstacle
        gradient = (yf-yi)/(xf-xi) #TODO: check for division by zero

        collision_left = self.y0 <= gradient*(self.x0-xi) + yi <= self.y1
        collision_right = self.y0 <= gradient*(self.x1-xi) + yi <= self.y1
        collision_top = self.x0 <= (self.y0 - yi) / gradient + xi <= self.x1
        collision_bottom = self.x0 <= (self.y1 - yi) / gradient + xi <= self.x1
        return collision_left or collision_right or collision_top or collision_bottom
    
    def detect_contained(self, xi, yi):
        '''
        Check if a point (xi,yi) is contained in the obstacle.
        '''
        return self.x0 <= xi <= self.x1 and self.y0 <= yi <= self.y1
    
def rrt(x_min,x_max,y_min,y_max,start_coords:list,gate_coords:list, keep_out_boxes:list):
    '''
    RRT algorithm to find a path sequentially from (x0,y0) to (xn,yn).
    '''
    # Create the list of mandatory waypoints
    gates = gate_coords.copy()
    # Create the root node
    root = TreeNode(start_coords[0], start_coords[1])

    # Create the tree
    tree = [root]
    # Create the list of keep out boxes
    keep_out_boxes = [KeepOutBox(*box) for box in keep_out_boxes]
    # Create list of orphans
    orphans = []

    i = 0
    flag_gate_loaded = False
    while i < 1e8:
        i += 1
        try:
            # Put next gate at head of orphans list
            if not flag_gate_loaded:
                try:
                    next_gate = gates.pop(0)
                    flag_gate_loaded = True
                    orphans = [TreeNode(next_gate[0], next_gate[1])] + orphans
                except IndexError:
                    # No more gates to process
                    break
            
            all_nodes = tree + orphans

            # Randomly sample a point in the extent of all nodes + 1 turning radius around
            # node_distances = [node.get_distance(start_coords[0],start_coords[1]) for node in all_nodes]
            # max_distance = max(node_distances)
            # x_min = max(start_coords[0] - max_distance, x_min)
            # x_max = min(start_coords[0] + max_distance, x_max)
            # y_min = max(start_coords[1] - max_distance, y_min)
            # y_max = min(start_coords[1] + max_distance, y_max)
            xk = np.random.uniform(x_min - 2*AVG_TURN_RADIUS, x_max + 2*AVG_TURN_RADIUS)
            yk = np.random.uniform(y_min - 2*AVG_TURN_RADIUS, y_max + 2*AVG_TURN_RADIUS)
            # print(f"Sampled ({xk: 05.1f},{yk: 05.1f})\r", end="")

            # Check if the point is inside any keep out boxes
            if any([box.detect_contained(xk,yk) for box in keep_out_boxes]):
                # Point is inside a keep out box, skip this iteration
                continue

            min_deviation = np.inf
            min_index = -1
            for j, node in enumerate(all_nodes):
                fwd_deviation = node.get_fwd_deviation(xk,yk)
                if fwd_deviation < min_deviation:
                    min_deviation = fwd_deviation
                    min_index = j

            if min_index == -1:
                # No nodes in the tree 
                # Create node in orphans list
                orphans.append(TreeNode(xk,yk))
                continue

            # Check if forward deviation is within limits
            if min_deviation < MAX_DEVIATION:
                # Check if the line segment intersects with any keep out boxes
                collision_list = [
                    box.detect_collision(
                        tree[min_index].get_fwd_position()[0], 
                        tree[min_index].get_fwd_position()[1], 
                        xk, 
                        yk
                    ) for box in keep_out_boxes]
                
                if any(collision_list):
                    # Collision detected, assign to orphans
                    orphans.append(TreeNode(xk,yk))
                    
                else:
                    # Create a new node and add it to the tree
                    new_node = TreeNode(xk,yk)
                    new_node.add_parent(tree[min_index])
                    
                    newly_adopted = []
                    newly_adopted.append(new_node)

                    while len(newly_adopted):
                        newly_adopted_node = newly_adopted.pop(0)
                        tree.append(newly_adopted_node)
                        print(f"Adopted node {newly_adopted_node.get_position()} from orphans")
                        # Check if any orphans are eligible for adoption
                        for k,orphan in enumerate(orphans):
                            if newly_adopted_node.get_fwd_deviation(
                                orphan.get_position()[0], 
                                orphan.get_position()[1]
                            ) < MAX_DEVIATION:
                                # Check if the line segment intersects with any keep out boxes
                                collision_list = [
                                    box.detect_collision(
                                        newly_adopted_node.get_fwd_position()[0], 
                                        newly_adopted_node.get_fwd_position()[1], 
                                        orphan.get_position()[0], 
                                        orphan.get_position()[1]
                                    ) for box in keep_out_boxes
                                ]
                                
                                if not any(collision_list):
                                    # No collision, adopt the orphan
                                    orphan.add_parent(newly_adopted_node)
                                    newly_adopted.append(orphans.pop(k))

                # Check if next gate was just adopted
                if tuple(next_gate) in [node.get_position() for node in tree]:
                    flag_gate_loaded = False

                print(f"Iteration {i}: Sampled ({xk: 05.1f},{yk: 05.1f}) from ({x_min},{y_min}) to ({x_max},{y_max}), {len(tree): 5d} nodes in tree, {len(orphans) :5d} orphans, next gate is {next_gate}")
        except KeyboardInterrupt:
            print("Stopping RRT algorithm")
            break

    # Calculate finishing path and time
    path = tree[-1].get_path()
    time = TIMESTEP_SEC * len(path)

    return tree, orphans, path, time

def plot_tree(gate_coords, tree, orphans, path, keep_out_boxes):
    '''
    Animate the growth of the tree.
    '''
    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    ax.set_title('RRT Tree Growth')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid()
    # Add keep out boxes to the plot
    for box in keep_out_boxes:
        rect = patches.Rectangle((box[0], box[2]), box[1]-box[0], box[3]-box[2], linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
    # Add the tree to the plot
    for node in tree:
        if node.get_parent() is not None:
            parent_x, parent_y = node.get_parent().get_position()
            line = lines.Line2D([parent_x, node._x], [parent_y, node._y], color='b', linewidth=1)
            ax.add_line(line)
    for orphan in orphans:
        orphan_x, orphan_y = orphan.get_position()
        ax.plot(orphan_x, orphan_y, 'rx', markersize=5)
        # print(f"Orphan node {orphan.get_position()}")

    # Add the start and goal nodes to the plot
    start_node = tree[0]
    gate_coords = np.array(gate_coords)
    ax.plot(start_node._x, start_node._y, 'go', markersize=10, label='Start')
    ax.plot(gate_coords[:,0], gate_coords[:,1], 'ro', markersize=10, label='Goal', )

    # Add the path to the plot
    path_x = [node._x for node in path]
    path_y = [node._y for node in path]
    ax.plot(path_x, path_y, 'r-', linewidth=4, label='Path')

    ax.legend()
    plt.show()

def main():
    # Define the limits of the space
    x_min = 0
    x_max = 10
    y_min = 0
    y_max = 10

    # Define the start and goal coordinates
    start_coords = [0, 0]
    gate_coords = [[2, 2], [5, 5], [8, 8]]
    gate_coords_copy = gate_coords.copy()

    # Define the keep out boxes
    keep_out_boxes = [
        [3, 4, 3, 4],
        [6, 7, 6, 7]
    ]

    # Run the RRT algorithm
    tree, orphans, path, time = rrt(x_min,x_max,y_min,y_max,start_coords,gate_coords,keep_out_boxes)

    print(f"Path found taking {time:.2f} seconds")

    # plot the tree
    plot_tree(gate_coords_copy,tree, orphans, path, keep_out_boxes)

if __name__ == "__main__":
    main()










