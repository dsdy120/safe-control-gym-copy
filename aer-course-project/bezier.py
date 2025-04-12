"""Example utility module.

Please use a file like this one to add extra functions.

"""

class Trajectory:
    def __init__(self, gate_coords):
        self._gate_coords = gate_coords
        self._curves = []


        self._path = []

class Obstacle:
    def __init__(self, x0, y0, x1, y1):
        self._x0 = x0
        self._x1 = x1
        self._y0 = y0
        self._y1 = y1

    def check_collision(self, x, y):
        if (self._x0 <= x <= self._x1) and (self._y0 <= y <= self._y1):
            return True
        return False
    
class BezierCurve:
    def __init__(self, p0, p1, p2, p3):
        self._p0 = p0
        self._p1 = p1
        self._p2 = p2
        self._p3 = p3

    def get_point(self, t):
        x = (1-t)**3 * self._p0[0] + 3*(1-t)**2 * t * self._p1[0] + 3*(1-t) * t**2 * self._p2[0] + t**3 * self._p3[0]
        y = (1-t)**3 * self._p0[1] + 3*(1-t)**2 * t * self._p1[1] + 3*(1-t) * t**2 * self._p2[1] + t**3 * self._p3[1]
        return (x, y)
    
    def shift_p1(self, dx, dy):
        self._p1 = (self._p1[0] + dx, self._p1[1] + dy)

    def shift_p2(self, dx, dy):
        self._p2 = (self._p2[0] + dx, self._p2[1] + dy)

    def set_p1(self, p1):
        self._p1 = p1

    def set_p2(self, p2):
        self._p2 = p2

class Gate:
    def __init__(self, x0, y0, x1, y1, vertical, previous_curve, next_curve):
        self._x0 = x0
        self._x1 = x1
        self._y0 = y0
        self._y1 = y1
        self._vertical = vertical
        self._previous_curve = previous_curve
        self._next_curve = next_curve

    def shift_ctrl_points(self, dx, dy):
        self._previous_curve.shift_p2(-dx, -dy)
        self._next_curve.shift_p1(dx, dy)
