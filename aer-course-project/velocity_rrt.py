'''
RRT algorithm
'''
import numpy as np
import matplotlib.pyplot as plt

TIMESTEP_SEC = 0.1

DIST_COEFF = 1
DIST_EXP = 1
VEL_COEFF = 1
VEL_EXP = 0.00

TOLERANCE_COST = 0.5

MAX_ITERATIONS = 50000000

class RootNode:
    def __init__(self,x:np.ndarray,x_f:np.ndarray,v_f:np.ndarray):
        self._x = x
        self._parent = None
        self._cost = np.linalg.norm(x - x_f) + np.linalg.norm(v_f)
        self._vel = np.zeros(3)

    def get_future_x(self):
        return self._x + self._vel * TIMESTEP_SEC
    def get_x(self):
        return self._x
    def get_vel(self):
        return self._vel
    def get_parent(self):
        return self._parent
    def get_cost(self,x_f,v_f):
        return self._cost
    def get_x_path(self):
        return [self._x]
    def get_v_path(self):
        return [self._vel]

class TreeNode:
    def __init__(self, parent, x:np.ndarray, x_f:np.ndarray, v_f:np.ndarray):
        self._x = x
        self._parent = parent

        parent_rel_x = np.array(self._x) - np.array(self._parent.get_x())
        self._vel = parent_rel_x / TIMESTEP_SEC

        self._cost = self.get_cost(x_f,v_f)

    def get_future_x(self):
        return self._x + self._vel * TIMESTEP_SEC

    def get_x(self):
        return self._x
    
    def get_vel(self):
        return self._vel

    def get_parent(self):
        return self._parent
    
    def get_cost(self,x_f,v_f):
        return DIST_COEFF*np.linalg.norm(self.get_future_x() - x_f)**DIST_EXP * VEL_COEFF*np.linalg.norm(self._vel - v_f)**VEL_EXP

    def get_x_path(self):
        path = []
        node = self
        while node is not None:
            path.append(node.get_x())
            print(path[-1])
            node = node._parent
        return path[::-1]
    
    def get_v_path(self):
        path = []
        node = self
        while node is not None:
            path.append(node.get_vel())
            node = node._parent
        return path[::-1]


def rrt(x_0,x_f,v_f,obstacles,max_acc):
    """
    RRT algorithm for path planning.
    """
    # Initialize the tree with the start node
    start_root = RootNode(x_0,x_f,v_f)
    fwd_tree = [start_root]
    # min_cost = root.get_cost()
    start_root_cost = start_root.get_cost(x_f,v_f)
    min_fwd_cost = start_root_cost

    goal_root = RootNode(x_f,x_0,0)
    bwd_tree = [TreeNode(goal_root,x_f-v_f*TIMESTEP_SEC,x_0,0)]
    goal_root_cost = goal_root.get_cost(x_0,0)
    min_bwd_cost = goal_root_cost
    best_fwd_node = fwd_tree[-1]
    best_bwd_node = bwd_tree[-1]

    for i in range(MAX_ITERATIONS):
        min_fwd_cost = min(min_fwd_cost, best_fwd_node.get_cost(best_bwd_node.get_x(),best_bwd_node.get_vel()))
        min_bwd_cost = min(min_bwd_cost, best_bwd_node.get_cost(best_fwd_node.get_x(),best_fwd_node.get_vel()))
        try:
            # Sort the tree by descending cost
            fwd_tree.sort(key=lambda node: node.get_cost(best_bwd_node.get_x(),best_bwd_node.get_vel()), reverse=True)
            print("Iteration:", i, "Tree Size: ", len(fwd_tree) ,"Cost:", min_fwd_cost, "X:", fwd_tree[-1].get_x(), "V:", fwd_tree[-1].get_vel())

            bwd_tree.sort(key=lambda node: node.get_cost(best_fwd_node.get_x(),best_fwd_node.get_vel()), reverse=True)
            print("Iteration:", i, "Tree Size: ", len(bwd_tree) ,"Cost:", min_bwd_cost, "X:", bwd_tree[-1].get_x(), "V:", bwd_tree[-1].get_vel())
            best_fwd_node = fwd_tree[-1]
            best_bwd_node = bwd_tree[-1]

            for j in range(500):
                a_rand = np.array([max_acc,max_acc])
                while np.linalg.norm(a_rand) > max_acc:
                    a_rand = np.random.uniform(-max_acc, max_acc, 2)
                
                a_rand = np.array([a_rand[0], a_rand[1], 0])

                # rnd_index = min(len(tree)-1,int(np.e**(np.random.random()*np.log(len(tree)))))
                # print(rnd_index)
                x_step = 0.5 * a_rand * TIMESTEP_SEC**2
                fwd_tree_node:TreeNode = fwd_tree[-1-j]
                new_fwd_tree_node = TreeNode(fwd_tree_node, fwd_tree_node.get_future_x() + x_step,bwd_tree[-1].get_x(),-bwd_tree[-1].get_vel())

                fwd_tree.append(new_fwd_tree_node)
        
                a_rand = np.array([max_acc,max_acc])
                while np.linalg.norm(a_rand) > max_acc:
                    a_rand = np.random.uniform(-max_acc, max_acc, 2)
                
                a_rand = np.array([a_rand[0], a_rand[1], 0])

                x_step = 0.5 * a_rand * TIMESTEP_SEC**2
                bwd_tree_node:TreeNode = bwd_tree[-1-j]
                new_bwd_tree_node = TreeNode(bwd_tree_node, bwd_tree_node.get_future_x() + x_step,fwd_tree[-1].get_x(),fwd_tree[-1].get_vel())

                bwd_tree.append(new_bwd_tree_node)

            # Check if the two trees are close enough to connect
            if np.linalg.norm(fwd_tree[-1].get_x() - bwd_tree[-1].get_x()) < TOLERANCE_COST:
                print("Trees connected")
                break

        except KeyboardInterrupt:
            print("Interrupted")
            break

    fwd_x_path = fwd_tree[-1].get_x_path()
    fwd_v_path = fwd_tree[-1].get_v_path()

    bwd_x_path = bwd_tree[-1].get_x_path()
    bwd_v_path = bwd_tree[-1].get_v_path()
    
    return fwd_x_path + bwd_x_path[::-1], fwd_v_path + bwd_v_path[::-1]


if __name__ == "__main__":
    x_0 = np.array([0, 0, 0])
    x_f = np.array([10, 10, 0])
    v_f = np.array([0, 1, 0])
    obstacles = []
    max_acc = 2
    goal_sample_rate = 0.05
    max_distance = 1.0

    path_x, path_v = rrt(x_0,x_f,v_f,obstacles,max_acc)
    

    # Plot the path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([x[0] for x in path_x], [x[1] for x in path_x], [x[2] for x in path_x], color='blue')
    ax.plot([x[0] for x in path_v], [x[1] for x in path_v], [x[2] for x in path_v], color='red')
    ax.scatter(x_f[0], x_f[1], x_f[2], color='green', marker='o', label='Goal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('RRT Path')
    plt.show()

