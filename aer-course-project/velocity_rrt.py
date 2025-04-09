'''
RRT algorithm
'''


import numpy as np

def rrt(x_0,v_0,x_f,v_f,dt,a_max, max_iter=10000):
    """
    RRT algorithm to find a path from (x_0, v_0) to (x_f, v_f) in a given time step dt.
    """
    # Initialize the tree with the start node
    tree = [(x_0, v_0)]
    
    for i in range(max_iter):
        # Sample a random acceleration vector
        a = np.array([a_max, a_max])
        while np.linalg.norm(a) > a_max:
            # Sample a random acceleration vector
            a = np.random.uniform(-1, 1, size=(2,))

    # Calculate the new velocity and position
        v_new = v_0 + a * dt
        x_new = x_0 + v_0 * dt + 0.5 * a * dt**2

        # Check if the new position and velocity are within bounds
        if np.linalg.norm(x_new - x_f) < 1e-3 and np.linalg.norm(v_new - v_f) < 1e-3:
            return True, (x_new, v_new)

        # Add the new node to the tree
        tree.append((x_new, v_new))

def cost_fn(x_k,v_k,x_f,v_f,dt):
    dist_after_dt = np.linalg.norm(x_k + v_k*dt - x_f)
    dv = np.linalg.norm(v_k - v_f)

    return dist_after_dt + dv