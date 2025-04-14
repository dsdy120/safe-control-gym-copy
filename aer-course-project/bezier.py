"""Example utility module.

Please use a file like this one to add extra functions.

"""
import numpy as np

class Trajectory:
    def __init__(self, gate_coords:list, ko_box_coords:list, ):
        self._gate_coords = gate_coords
        self._curves = []
        self._ko_boxes = []
        self._gates = []

        for coord_triplet in gate_coords:
            flag_vertical, x, y = coord_triplet
            self._gates.append(Gate((x,y), flag_vertical))

        for i in range(len(gate_coords)-1):
            curr_gate:Gate = self._gates[i]
            next_gate:Gate = self._gates[i+1]

            self._curves.append(BezierCurve(
                (0,0),
                (0,0),
                (0,0),
                (0,0)
            ))

            curr_gate.register_next_curve(self._curves[i])
            next_gate.register_prev_curve(self._curves[i])

        for ko_box in ko_box_coords:
            x0, y0, x1, y1 = ko_box
            self._ko_boxes.append(KeepOutBox(x0, y0, x1, y1))

    def get_gate_coords(self,gate_index)->tuple:
        if gate_index < 0 or gate_index >= len(self._gates):
            raise ValueError("gate index out of range")
        return self._gates[gate_index].get_gate_location()

class KeepOutBox:
    def __init__(self, x0:float, y0:float, x1:float, y1:float):
        self._x0 = x0
        self._x1 = x1
        self._y0 = y0
        self._y1 = y1

    def check_collision(self, x, y):
        if (self._x0 <= x <= self._x1) and (self._y0 <= y <= self._y1):
            return True
        return False
    
class BezierCurve:
    def __init__(self, p0:tuple, p1:tuple, p2:tuple, p3:tuple):
        self._p0 = p0
        self._p1 = p1
        self._p2 = p2
        self._p3 = p3

    def get_point(self, t):
        x = (1-t)**3 * self._p0[0] + 3*(1-t)**2 * t * self._p1[0] + 3*(1-t) * t**2 * self._p2[0] + t**3 * self._p3[0]
        y = (1-t)**3 * self._p0[1] + 3*(1-t)**2 * t * self._p1[1] + 3*(1-t) * t**2 * self._p2[1] + t**3 * self._p3[1]
        return (x, y)
    
    def get_p0(self):
        return self._p0

    def get_p1(self):
        return self._p1
    
    def get_p2(self):
        return self._p2
    
    def get_p3(self):
        return self._p3
    
    def set_p0(self, p0):
        self._p0 = p0

    def set_p1(self, p1):
        self._p1 = p1

    def set_p2(self, p2):
        self._p2 = p2

    def set_p3(self, p3):
        self._p3 = p3

    def check_collision_fraction(self,ko_box:KeepOutBox)->float:
        '''
        Checks the proportion of the bezier curve that is inside the ko_box
        '''
        num_points = 1000
        fraction = 0
        for t in range(num_points+1):
            t /= num_points
            x, y = self.get_point(t)
            if ko_box.check_collision(x, y):
                fraction += 1/num_points
        return fraction

class Gate:
    def __init__(self, coords:tuple, vertical:bool, previous_curve:BezierCurve=None, next_curve:BezierCurve=None):
        self._X:float = coords[0]
        self._Y:float = coords[1]
        self._VERTICAL = vertical

        if next_curve is None:
            next_curve = BezierCurve((self._X, self._Y), (self._X, self._Y), (self._X, self._Y), (self._X, self._Y))
        if previous_curve is None:
            previous_curve = BezierCurve((self._X, self._Y), (self._X, self._Y), (self._X, self._Y), (self._X, self._Y))

        self.register_next_curve(next_curve)
        self.register_prev_curve(previous_curve)

    def get_gate_location(self):
        return (self._X, self._Y)

    def _get_next_p1_abs(self):
        if self._next_curve is not None:
            return self._next_curve.get_p1()
        else:
            return (self._X, self._Y)
        
    def _get_prev_p2_abs(self):
        if self._previous_curve is not None:
            return self._previous_curve.get_p2()
        else: 
            return (self._X, self._Y)
        
    def register_prev_curve(self, curve:BezierCurve):
        self._previous_curve = curve
        self._previous_curve.set_p3((self._X, self._Y))
        rel_x = self._X + 1 if self._VERTICAL else self._X
        rel_y = self._Y + 1 if not self._VERTICAL else self._Y
        self.set_ctrl_points_rel(rel_x, rel_y)

    def register_next_curve(self, curve:BezierCurve):
        self._next_curve = curve
        self._next_curve.set_p0((self._X, self._Y))
        abs_x = self._X + 1 if self._VERTICAL else self._X
        abs_y = self._Y + 1 if not self._VERTICAL else self._Y
        self.set_ctrl_points_abs(abs_x, abs_y)

    def set_ctrl_points_abs(self, abs_x:float, abs_y:float):
        self._previous_curve.set_p2((2*self._X - abs_x , 2*self._Y - abs_y))
        self._next_curve.set_p1((abs_x, abs_y))
        
    def get_point(self, t):
        if t < -1 or t > 1:
            raise ValueError("t must be in the range [-1, 1]")
        if t < 0:
            t = 1+t
            return self._previous_curve.get_point(t)
        else:
            return self._next_curve.get_point(t)
        
    def aligned_ctrl_pt_rnd_walk(self,max_shift:float, bias:float=0):
        if self._VERTICAL:
            self.set_ctrl_points_abs(self._X + bias + np.random.uniform(-max_shift, max_shift), self._Y)
        else:
            self.set_ctrl_points_abs(self._X, self._Y + bias + np.random.uniform(-max_shift, max_shift))