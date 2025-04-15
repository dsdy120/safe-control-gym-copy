"""Example utility module.

Please use a file like this one to add extra functions.

"""
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from matplotlib import colors
from matplotlib import cm
import collections

AVG_COLLISION_THRESHOLD = 0.002
SAMPLE_BUFFER_SIZE = 1
RANDOM_WALK_RANGE = 1
MIN_CTRL_DIST = 0.5

def animation(x_lim, y_lim, gate_coords:list, ko_box_coords:list, traj_history:list, interval=16):
    """
    Generates an animation of the trajectory optimization process with a frame counter.
    The `interval` parameter controls the speed of the animation (in milliseconds).
    """
    fig, ax = plt.subplots()
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)
    ax.set_aspect('equal', adjustable='box')

    # Create a list of patches for the gates
    gate_patches = []
    for gate in gate_coords:
        x, y = gate[1], gate[2]
        if gate[0]:
            rect = Rectangle((x-0.125, y-0.125), 0.25, 0.25, color='red')
        else:
            rect = Rectangle((x-0.125, y-0.125), 0.25, 0.25, color='red')
        gate_patches.append(rect)

    # Create a list of patches for the keep-out boxes
    ko_box_patches = []
    for ko_box in ko_box_coords:
        x0, x1, y0, y1 = ko_box
        rect = Rectangle((x0, y0), x1-x0, y1-y0, color='green', alpha=0.5)
        ko_box_patches.append(rect)

    # Create a collection of all patches
    all_patches = PatchCollection(gate_patches + ko_box_patches)

    # Add the patches to the axes
    ax.add_collection(all_patches)

    # Create a line object for the trajectory
    line, = ax.plot([], [], lw=2, color = 'red')

    # Add a text object for the frame counter
    frame_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, color='black')

    def init():
        line.set_data([], [])
        frame_text.set_text('')
        return line, frame_text

    def animate(i):
        if i < len(traj_history):
            # Each step of the animation is a sequence of Bezier curves
            # Plot the trajectory in each step by stringing the Bezier curves together
            x_data = []
            y_data = []
            for curve in traj_history[i]:
                bezier_curve = BezierCurve(*curve)
                for t in np.linspace(0, 1, 100):
                    x, y = bezier_curve.get_point(t)
                    x_data.append(x)
                    y_data.append(y)

            line.set_data(x_data, y_data)
            frame_text.set_text(f"Frame: {i+1}/{len(traj_history)}")
            return line, frame_text
        else:
            # Pause on the last frame
            ani.event_source.stop()
            return line, frame_text
    
    ani = FuncAnimation(fig, animate, frames=len(traj_history), init_func=init, blit=True, repeat=False, interval=interval)
    plt.show()



class Trajectory:
    def __init__(self, gate_coords:list, ko_box_coords:list, ):
        self._gate_coords = gate_coords
        self._curves = []
        self._ko_boxes = []
        self._gates = []
        self._traj_history = []
        self._GATE_SIGNS = []

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

        # Set gate control pts to lead to the next gate organically
        for i in range(len(self._curves)):
            curr_gate:Gate = self._gates[i]
            next_gate:Gate = self._gates[i+1]

            curr_locn = curr_gate.get_gate_location()
            next_locn = next_gate.get_gate_location()

            if curr_gate._VERTICAL:
                sign = np.sign(next_locn[0] - curr_locn[0])
                self._GATE_SIGNS.append(sign)
                curr_gate.set_ctrl_points_abs(
                    curr_locn[0] + MIN_CTRL_DIST * sign,
                    curr_locn[1]
                )
            else:
                sign = np.sign(next_locn[1] - curr_locn[1])
                self._GATE_SIGNS.append(sign)
                curr_gate.set_ctrl_points_abs(
                    curr_locn[0],
                    curr_locn[1] + MIN_CTRL_DIST * sign
                )

        self._gates[-1].set_ctrl_points_abs(*self._gates[-1].get_gate_location())

        for i in range(len(self._curves)):
            print(self._gates[i].get_next_p1_abs()[0] - self._gates[i].get_gate_location()[0], end=", ")
            print(self._gates[i].get_next_p1_abs()[1] - self._gates[i].get_gate_location()[1])

        for ko_box in ko_box_coords:
            self._ko_boxes.append(KeepOutBox(*ko_box))

    def get_gate_coords(self,gate_index)->tuple:
        if gate_index < 0 or gate_index >= len(self._gates):
            raise ValueError("gate index out of range")
        return self._gates[gate_index].get_gate_location()
    
    def optimize_trajectory(self):
        collision_fractions = collections.deque(maxlen=SAMPLE_BUFFER_SIZE)
        for i in range(SAMPLE_BUFFER_SIZE):
            collision_fractions.appendleft(1)

        gate_init_signs = self._GATE_SIGNS.copy()

        best_avg_collision_fraction = 1.0
        collision_delta = 1.0
        best_ctrl_deviation = [0.0 for gate in self._gates]
        ctrl_cfg_deviation = [0.0 for gate in self._gates]
        t_start = time.perf_counter()
        while False:
            if time.perf_counter() - t_start > 15:
                print("Trajectory optimization timed out.")
                break
            try:
                for i, gate in enumerate(self._gates):
                    gate:Gate
                    gate_x = gate.get_gate_location()[0]
                    gate_y = gate.get_gate_location()[1]
                    gate_sign = gate_init_signs[i]
                    if gate._VERTICAL:
                        if gate_sign > 0:
                            gate.set_ctrl_points_abs(
                                max(
                                    gate_x + best_ctrl_deviation[i]\
                                        + np.random.uniform(-RANDOM_WALK_RANGE, RANDOM_WALK_RANGE)
                                    ,(1)*MIN_CTRL_DIST
                                )
                                ,gate_y
                            )
                        else:
                            gate.set_ctrl_points_abs(
                                min(
                                    gate_x + best_ctrl_deviation[i]\
                                        + np.random.uniform(-RANDOM_WALK_RANGE, RANDOM_WALK_RANGE)
                                    ,(-1)*MIN_CTRL_DIST
                                )
                                ,gate_y
                            )
                    else:
                        if gate_sign > 0:
                            gate.set_ctrl_points_abs(
                                gate_x
                                ,max(
                                    gate_y + best_ctrl_deviation[i]\
                                        + np.random.uniform(-RANDOM_WALK_RANGE, RANDOM_WALK_RANGE)
                                    ,(1) * MIN_CTRL_DIST
                                )
                            )
                        else:
                            gate.set_ctrl_points_abs(
                                gate_x
                                ,min(
                                    gate_y + best_ctrl_deviation[i]\
                                        + np.random.uniform(-RANDOM_WALK_RANGE, RANDOM_WALK_RANGE)
                                    ,(-1)*MIN_CTRL_DIST
                                )
                            )

                indiv_collision_fractions = [
                    curve.check_collision_fraction(ko_box)
                    for curve in self._curves
                    for ko_box in self._ko_boxes
                ]

                new_collision_fraction = sum(indiv_collision_fractions) / len(indiv_collision_fractions)
                collision_fractions.appendleft(new_collision_fraction)

                self._traj_history.append(
                    [curve.get_all_ctrl_points() for curve in self._curves]
                )

                avg_collision_fraction = sum(collision_fractions) / len(collision_fractions)
                collision_delta = avg_collision_fraction - best_avg_collision_fraction
                if collision_delta < 0:
                    best_ctrl_deviation = ctrl_cfg_deviation.copy()
                    best_avg_collision_fraction = avg_collision_fraction


                print(f"Average collision fraction: {avg_collision_fraction: .4f}, Collision delta: {collision_delta: .4f}\r", end="")
                if avg_collision_fraction < AVG_COLLISION_THRESHOLD:
                    break

            except KeyboardInterrupt:
                print("Trajectory optimization interrupted.")
                break

        return self._traj_history
    
    def get_trajectory(self):
        trajectory = []
        for curve in self._curves:
            # interpolate between points
            for t in np.linspace(0, 1, 50):
                x, y = curve.get_point(t)
                trajectory.append([x, y])
        return trajectory


class KeepOutBox:
    def __init__(self, x0:float, x1:float, y0:float, y1:float):
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
    
    def get_all_ctrl_points(self):
        return [self._p0, self._p1, self._p2, self._p3]

class Gate:
    def __init__(self, coords:tuple, vertical:bool, previous_curve:BezierCurve=None, next_curve:BezierCurve=None):
        self._X:float = coords[0]
        self._Y:float = coords[1]
        self._VERTICAL = vertical

        self._next_curve = BezierCurve((self._X, self._Y), (self._X, self._Y), (self._X, self._Y), (self._X, self._Y))
        self._previous_curve = BezierCurve((self._X, self._Y), (self._X, self._Y), (self._X, self._Y), (self._X, self._Y))

        if next_curve is not None:
            self.register_next_curve(next_curve)
        if previous_curve is not None:
            self.register_prev_curve(previous_curve)

    def get_gate_location(self):
        return (self._X, self._Y)

    def get_next_p1_abs(self):
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
        self.set_ctrl_points_abs(self._X + rel_x, self._Y + rel_y)

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
        step = np.random.uniform(-max_shift, max_shift) + bias
        if self._VERTICAL:
            self.set_ctrl_points_abs(self._X + step, self._Y)
        else:
            self.set_ctrl_points_abs(self._X, self._Y + step)

        return step
    
def main():
    gate_coords = [(True, 0, 0), (False, 2, 2), (True, 4, 4), (False, 2, 2)] * 2
    ko_box_coords = [(0.5, 0.5, 1.5, 1.5), (2.5, 2.5, 3.5, 3.5)]
    # Add wall hit-boxes 20 units deep on all 4 sides of the play area, defined by -1,-1 and 5,5
    ko_box_coords.append((-20, -20, -1, 20)) # left wall
    ko_box_coords.append((5, -20, 25, 20)) # right wall
    ko_box_coords.append((-20, -21, 20, -1)) # bottom wall
    ko_box_coords.append((-20, 5, 20, 25)) # top wall

    x_lim = (-1, 5)
    y_lim = (-1, 5)
    traj = Trajectory(gate_coords, ko_box_coords)
    traj_history = traj.optimize_trajectory()
    animation(x_lim, y_lim, gate_coords, ko_box_coords, traj_history)

if __name__ == "__main__":
    main()