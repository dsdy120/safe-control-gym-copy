'''
RRT algorithm
'''
import numpy as np
import matplotlib.pyplot as plt

TIMESTEP_SEC = 0.1

DIST_COEFF = 1
DIST_EXP = 10
VEL_COEFF = 1
VEL_EXP = 1

TOLERANCE_COST = 0

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
    def get_cost(self):
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

        self._cost = DIST_COEFF*np.linalg.norm(self.get_future_x() - x_f)**DIST_EXP + VEL_COEFF*np.linalg.norm(self._vel - v_f)**VEL_EXP

    def get_future_x(self):
        return self._x + self._vel * TIMESTEP_SEC

    def get_x(self):
        return self._x
    
    def get_vel(self):
        return self._vel

    def get_parent(self):
        return self._parent
    
    def get_cost(self):
        return self._cost

    def get_x_path(self):
        path = []
        node = self
        while node is not None:
            path.append(node.get_x())
            node = node._parent
        return path[::-1]
    
    def get_v_path(self):
        path = []
        node = self
        while node is not None:
            path.append(node.get_vel())
            node = node._parent
        return path[::-1]


def rrt(x_0,x_1,x_f,v_f,obstacles,max_acc):
    """
    RRT algorithm for path planning.
    """
    # Initialize the tree with the start node
    root = RootNode(x_0,x_1,x_f)
    tree = [root]
    # min_cost = root.get_cost()
    root_cost = root.get_cost()
    min_cost = root_cost

    for i in range(MAX_ITERATIONS):
        min_cost = min(min_cost, tree[-1].get_cost())
        j = 0
        try:
            # if tree[-1-j].get_cost() > 1.0005 * min_cost:
            #     continue
            # Sample a random acceleration vector
            # for j in range(max(1000,int(min_cost / (root_cost) * len(tree)))):
            for j in range(10000):
                a_rand = np.array([max_acc,max_acc])
                while np.linalg.norm(a_rand) > max_acc:
                    a_rand = np.random.uniform(-max_acc, max_acc, 2)
                
                a_rand = np.array([a_rand[0], a_rand[1], 0])

                # rnd_index = min(len(tree)-1,int(np.e**(np.random.random()*np.log(len(tree)))))
                # print(rnd_index)
                x_step = 0.5 * a_rand * TIMESTEP_SEC**2
                tree_node:TreeNode = tree[-1-j]
                new_tree_node = TreeNode(tree_node, tree_node.get_future_x() + x_step,x_f,v_f)

                tree.append(new_tree_node)

                if new_tree_node.get_cost() < TOLERANCE_COST:
                    print("Goal reached")
                    return new_tree_node.get_x_path(), new_tree_node.get_v_path()
            
            # Sort the tree by descending cost
            tree.sort(key=lambda node: node.get_cost(), reverse=True)
            print("Iteration:", i, "Tree Size: ", len(tree) ,"Cost:", tree[-1].get_cost(), "X:", tree[-1].get_x(), "V:", tree[-1].get_vel())
        except KeyboardInterrupt:
            print("Interrupted")
            return tree[-1].get_x_path(), tree[-1].get_v_path()
        
    return tree[-1].get_x_path(), tree[-1].get_v_path()


if __name__ == "__main__":
    x_0 = np.array([0, 0, 0])
    x_f = np.array([10, 10, 0])
    v_f = np.array([1, -1, 0])
    obstacles = []
    max_acc = 2
    goal_sample_rate = 0.05
    max_distance = 1.0

    path_x, path_v = rrt(x_0,x_f,x_f,v_f,obstacles,max_acc)
    

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

