"""Example utility module.

Please use a file like this one to add extra functions.

"""

class Trajectory:
    def __init__(self, gate_coords:list, ko_box_coords:list, ):
        self._gate_coords = gate_coords
        self._curves = []
        self._ko_boxes = []
        self._gates = []

        for i in range(len(gate_coords)-1):
            p0 = gate_coords[i][1:]
            p1 = gate_coords[i+1][1:]
            self._curves.append(BezierCurve(p0, p0, p1, p1))
            #TODO: implement gate instantiationi and curve registration for each gate

        #TODO: Implement keep out box instantiation


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
    def __init__(self, p0, p1, p2, p3):
        self._p0 = p0
        self._p1 = p1
        self._p2 = p2
        self._p3 = p3

    def get_point(self, t):
        x = (1-t)**3 * self._p0[0] + 3*(1-t)**2 * t * self._p1[0] + 3*(1-t) * t**2 * self._p2[0] + t**3 * self._p3[0]
        y = (1-t)**3 * self._p0[1] + 3*(1-t)**2 * t * self._p1[1] + 3*(1-t) * t**2 * self._p2[1] + t**3 * self._p3[1]
        return (x, y)
    
    def get_p1(self):
        return self._p1
    
    def get_p2(self):
        return self._p2
    
    def set_p1(self, p1):
        self._p1 = p1

    def set_p2(self, p2):
        self._p2 = p2

    def check_collision(self,ko_box:KeepOutBox):
        # Check if the bezier curve intersects with the keep out box
        for t in range(0, 1001):
            t /= 1000
            x, y = self.get_point(t)
            if ko_box.check_collision(x, y):
                return True
        return False

class Gate:
    def __init__(self, coords:tuple, vertical:bool, previous_curve:BezierCurve=None, next_curve:BezierCurve=None):
        self._x:float = coords[0]
        self._y:float = coords[1]
        self._vertical = vertical
        self._previous_curve = previous_curve
        self._next_curve = next_curve
        self._next_p1:tuple = None
        self._previous_p2:tuple = self._next_p1

        # Set initial control points
        if vertical:
            self.set_ctrl_points_rel(0,1)
        else:
            self.set_ctrl_points_rel(1,0)

        if isinstance(self._previous_curve,BezierCurve):
            self._previous_p2 = self._previous_curve.get_p2()

        if isinstance(self._next_curve,BezierCurve):
            self._next_p1 = self._next_curve.get_p1()

    def _sync_curves(self):
        self._next_p1 = (self._next_curve._)
        self._previous_p2 = (self._x * 2, self._y * 2) - self._next_p1
        if self._next_curve is not None:
            self._next_curve.set_p1(self._next_p1)
        if self._previous_curve is not None:
            self._previous_curve.set_p2(self._previous_p2)

        #TODO: Add sync logic to all ctrl point methods

    def register_previous_curve(self, curve:BezierCurve):
        self._previous_curve = curve

    def register_next_curve(self, curve:BezierCurve):
        self._next_curve = curve

    def shift_ctrl_points(self,dx:float,dy:float):
        self._previous_curve.set_p2(())

    def set_ctrl_points_rel(self, rel_x:float, rel_y:float):
        self._previous_curve.set_p2((self._x - rel_x, self._y - rel_y))
        self._next_curve.set_p1((self._x + rel_x, self._y + rel_y))

    def set_ctrl_points_abs(self, abs_x:float, abs_y:float):
        self._previous_curve.set_p2((2*self._x - abs_x , 2*self._y - abs_y))
        self._next_curve.set_p1((abs_x, abs_y))
        
    def get_point(self, t):
        if t < -1 or t > 1:
            raise ValueError("t must be in the range [-1, 1]")
        if t < 0:
            t = 1+t
            return self._previous_curve.get_point(t)
        else:
            return self._next_curve.get_point(t)