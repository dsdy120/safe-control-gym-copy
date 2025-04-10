'''
RRT Waypoint Generator
'''

import numpy as np
import matplotlib.pyplot as plt



class TreeNode:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.children = []
        if parent is not None:
            parent.children.append(self)
        self.cost = 0.0
        self.path = []
        self.path.append(point)
        self.path_cost = 0.0

    def 