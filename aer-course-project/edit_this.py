"""Write your proposed algorithm.
[NOTE]: The idea for the final project is to plan the trajectory based on a sequence of gates 
while considering the uncertainty of the obstacles. The students should show that the proposed 
algorithm is able to safely navigate a quadrotor to complete the task in both simulation and
real-world experiments.

Then run:

    $ python3 final_project.py --overrides ./getting_started.yaml

Tips:
    Search for strings `INSTRUCTIONS` and `REPLACE THIS (START)` in this file.

    Change the code between the 5 blocks starting with
        #########################
        # REPLACE THIS (START) ##
        #########################
    and ending with
        #########################
        # REPLACE THIS (END) ####
        #########################
    with your own code.

    They are in methods:
        1) planning
        2) cmdFirmware

""" 
#comment
import numpy as np

from collections import deque

try:
    from project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory
except ImportError:
    # PyTest import.
    from .project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory

#########################
# REPLACE THIS (START) ##
#########################

# Optionally, create and import modules you wrote.
# Please refrain from importing large or unstable 3rd party packages.
try:
    import example_custom_utils as ecu
except ImportError:
    # PyTest import.
    from . import example_custom_utils as ecu

DURATION = 30

#########################
# REPLACE THIS (END) ####
#########################

class Controller():
    """Template controller class.

    """

    def __init__(self,
                 initial_obs,
                 initial_info,
                 use_firmware: bool = False,
                 buffer_size: int = 100,
                 verbose: bool = False
                 ):
        """Initialization of the controller.

        INSTRUCTIONS:
            The controller's constructor has access the initial state `initial_obs` and the a priori infromation
            contained in dictionary `initial_info`. Use this method to initialize constants, counters, pre-plan
            trajectories, etc.

        Args:
            initial_obs (ndarray): The initial observation of the quadrotor's state
                [x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r].
            initial_info (dict): The a priori information as a dictionary with keys
                'symbolic_model', 'nominal_physical_parameters', 'nominal_gates_pos_and_type', etc.
            use_firmware (bool, optional): Choice between the on-board controll in `pycffirmware`
                or simplified software-only alternative.
            buffer_size (int, optional): Size of the data buffers used in method `learn()`.
            verbose (bool, optional): Turn on and off additional printouts and plots.

        """
        # Save environment and control parameters.
        self.CTRL_TIMESTEP = initial_info["ctrl_timestep"]
        self.CTRL_FREQ = initial_info["ctrl_freq"]
        self.initial_obs = initial_obs
        self.VERBOSE = verbose
        self.BUFFER_SIZE = buffer_size

        # Store a priori scenario information.
        # plan the trajectory based on the information of the (1) gates and (2) obstacles. 
        self.NOMINAL_GATES = initial_info["nominal_gates_pos_and_type"]
        self.NOMINAL_OBSTACLES = initial_info["nominal_obstacles_pos"]

        # Check for pycffirmware.
        if use_firmware:
            self.ctrl = None
        else:
            # Initialize a simple PID Controller for debugging and test.
            # Do NOT use for the IROS 2022 competition. 
            self.ctrl = PIDController()
            # Save additonal environment parameters.
            self.KF = initial_info["quadrotor_kf"]

        # Reset counters and buffers.
        self.reset()
        self.interEpisodeReset()

        # perform trajectory planning
        t_scaled = self.planning(use_firmware, initial_info)

        ## visualization
        # Plot trajectory in each dimension and 3D.
        plot_trajectory(t_scaled, self.waypoints, self.ref_x, self.ref_y, self.ref_z)

        # Draw the trajectory on PyBullet's GUI.
        draw_trajectory(initial_info, self.waypoints, self.ref_x, self.ref_y, self.ref_z)


    def planning(self, use_firmware, initial_info):
        """Trajectory planning algorithm"""
        #########################
        # REPLACE THIS (START) ##
        #########################
        ## generate waypoints for planning

        # Call a function in module `example_custom_utils`.
        #ecu.exampleFunction()

        res = 0.1 # set resolution of the map
        obs = 1 # set obstacles (1:True, 0:False)
        # M = ecu.map_generation(res) # generate map with obstacles

        gate_order = np.array([4,2,3,1,4,2])
        # gate_order[-2:] = np.random.randint(1,5, size=2)
        # np.random.shuffle(gate_order)
        print("[Gate Order]:", gate_order)

        path, segments = ecu.path_planning(res, gate_order, obs).run_Astar()
        
        M = ecu.map_generation(res, obs)
        ecu.plot_map(M, res, path)

        # initial waypoint
        """if use_firmware:
            waypoints = [(self.initial_obs[0], self.initial_obs[2], initial_info["gate_dimensions"]["tall"]["height"])]  # Height is hardcoded scenario knowledge.
        else:
            waypoints = [(self.initial_obs[0], self.initial_obs[2], self.initial_obs[4])]

        # Example code: hardcode waypoints 
        waypoints.append((-0.5, -3.0, 2.0))
        waypoints.append((-0.5, -2.0, 2.0))
        waypoints.append((-0.5, -1.0, 2.0))
        waypoints.append((-0.5,  0.0, 2.0))
        waypoints.append((-0.5,  1.0, 2.0))
        waypoints.append((-0.5,  2.0, 2.0))
        waypoints.append([initial_info["x_reference"][0], initial_info["x_reference"][2], initial_info["x_reference"][4]])"""

        smooth_segments = []
        for seg in segments:
            real_path = np.array([[p[0]*res - 3.5, p[1]*res - 3.5, 1.0] for p in seg])

            deg = 100
            t = np.arange(real_path.shape[0])
            fx = np.poly1d(np.polyfit(t, real_path[:, 0], deg))
            fy = np.poly1d(np.polyfit(t, real_path[:, 1], deg))
            fz = np.poly1d(np.polyfit(t, real_path[:, 2], deg))

            t_smooth = np.linspace(t[0], t[-1], 50)
            x = fx(t_smooth)
            y = fy(t_smooth)
            z = fz(t_smooth)

            smooth_seg = np.vstack([x, y, z]).T
            smooth_segments.append(smooth_seg)

        smooth_path = np.vstack([s if i == 0 else s[1:] for i, s in enumerate(smooth_segments)])
        self.waypoints = smooth_path
        self._duration = DURATION
        t_scaled = np.linspace(0, smooth_path.shape[0]-1, int(self._duration*self.CTRL_FREQ))
        self.ref_x = np.interp(t_scaled, np.arange(smooth_path.shape[0]), smooth_path[:, 0])
        self.ref_y = np.interp(t_scaled, np.arange(smooth_path.shape[0]), smooth_path[:, 1])
        self.ref_z = np.interp(t_scaled, np.arange(smooth_path.shape[0]), smooth_path[:, 2])

        #########################
        # REPLACE THIS (END) ####
        #########################

        return t_scaled

    def cmdFirmware(self,
                    time,
                    obs,
                    reward=None,
                    done=None,
                    info=None
                    ):
        """Pick command sent to the quadrotor through a Crazyswarm/Crazyradio-like interface.

        INSTRUCTIONS:
            Re-implement this method to return the target position, velocity, acceleration, attitude, and attitude rates to be sent
            from Crazyswarm to the Crazyflie using, e.g., a `cmdFullState` call.

        Args:
            time (float): Episode's elapsed time, in seconds.
            obs (ndarray): The quadrotor's Vicon data [x, 0, y, 0, z, 0, phi, theta, psi, 0, 0, 0].
            reward (float, optional): The reward signal.
            done (bool, optional): Wether the episode has terminated.
            info (dict, optional): Current step information as a dictionary with keys
                'constraint_violation', 'current_target_gate_pos', etc.

        Returns:
            Command: selected type of command (takeOff, cmdFullState, etc., see Enum-like class `Command`).
            List: arguments for the type of command (see comments in class `Command`)

        """
        if self.ctrl is not None:
            raise RuntimeError("[ERROR] Using method 'cmdFirmware' but Controller was created with 'use_firmware' = False.")

        # [INSTRUCTIONS] 
        # self.CTRL_FREQ is 30 (set in the getting_started.yaml file) 
        # control input iteration indicates the number of control inputs sent to the quadrotor
        iteration = int(time*self.CTRL_FREQ)

        #########################
        # REPLACE THIS (START) ##
        #########################

        # print("The info. of the gates ")
        # print(self.NOMINAL_GATES)

        if iteration == 0:
            height = 1
            duration = 5

            command_type = Command(2)  # Take-off.
            args = [height, duration]
            print(f"Iteration: {iteration}, Command Type: {command_type}")

        # [INSTRUCTIONS] Example code for using cmdFullState interface   
        elif iteration >= 3*self.CTRL_FREQ and iteration < (self._duration + 3)*self.CTRL_FREQ:
            step = min(iteration-3*self.CTRL_FREQ, len(self.ref_x) -1)
            target_pos = np.array([self.ref_x[step], self.ref_y[step], self.ref_z[step]])
            target_vel = np.zeros(3)
            target_acc = np.zeros(3)
            target_yaw = 0.
            target_rpy_rates = np.zeros(3)

            command_type = Command(1)  # cmdFullState.
            args = [target_pos, target_vel, target_acc, target_yaw, target_rpy_rates]
            print(f"Iteration: {iteration}, Command Type: {command_type}")

        elif iteration == (self._duration+4)*self.CTRL_FREQ:
            command_type = Command(6)  # Notify setpoint stop.
            args = []
            print(f"Iteration: {iteration}, Command Type: {command_type}")

    #    # [INSTRUCTIONS] Example code for using goTo interface 
    #     elif iteration == 20*self.CTRL_FREQ+1:
    #         x = self.ref_x[-1]
    #         y = self.ref_y[-1]
    #         z = self.ref_z[-1]
    #         yaw = 0.
    #         duration = 30

    #         command_type = Command(5)  # goTo.
    #         args = [[x, y, z], yaw, duration, False]

        # elif iteration == 23*self.CTRL_FREQ:
        #     x = self.initial_obs[0]
        #     y = self.initial_obs[2]
        #     z = 1.5
        #     yaw = 0.
        #     duration = 6

        #     command_type = Command(5)  # goTo.
        #     args = [[x, y, z], yaw, duration, False]

        elif iteration == (self._duration+10)*self.CTRL_FREQ:
            height = 0.
            duration = 5

            command_type = Command(3)  # Land.
            args = [height, duration]
            print(f"Iteration: {iteration}, Command Type: {command_type}")

        elif iteration == (self._duration+18)*self.CTRL_FREQ:
            command_type = Command(4)  # STOP command to be sent once the trajectory is completed.
            args = []
            print(f"Iteration: {iteration}, Command Type: {command_type}")

        else:
            command_type = Command(0)  # None.
            args = []
            print(f"Iteration: {iteration}, Command Type: {command_type}")

        #########################
        # REPLACE THIS (END) ####
        #########################

        return command_type, args

    def cmdSimOnly(self,
                   time,
                   obs,
                   reward=None,
                   done=None,
                   info=None
                   ):
        """PID per-propeller thrusts with a simplified, software-only PID quadrotor controller.

        INSTRUCTIONS:
            You do NOT need to re-implement this method for the project.
            Only re-implement this method when `use_firmware` == False to return the target position and velocity.

        Args:
            time (float): Episode's elapsed time, in seconds.
            obs (ndarray): The quadrotor's state [x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r].
            reward (float, optional): The reward signal.
            done (bool, optional): Wether the episode has terminated.
            info (dict, optional): Current step information as a dictionary with keys
                'constraint_violation', 'current_target_gate_pos', etc.

        Returns:
            List: target position (len == 3).
            List: target velocity (len == 3).

        """
        if self.ctrl is None:
            raise RuntimeError("[ERROR] Attempting to use method 'cmdSimOnly' but Controller was created with 'use_firmware' = True.")

        iteration = int(time*self.CTRL_FREQ)

        #########################
        if iteration < len(self.ref_x):
            target_p = np.array([self.ref_x[iteration], self.ref_y[iteration], self.ref_z[iteration]])
        else:
            target_p = np.array([self.ref_x[-1], self.ref_y[-1], self.ref_z[-1]])
        target_v = np.zeros(3)
        #########################

        return target_p, target_v

    def reset(self):
        """Initialize/reset data buffers and counters.

        Called once in __init__().

        """
        # Data buffers.
        self.action_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.obs_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.reward_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.done_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.info_buffer = deque([], maxlen=self.BUFFER_SIZE)

        # Counters.
        self.interstep_counter = 0
        self.interepisode_counter = 0

    # NOTE: this function is not used in the course project. 
    def interEpisodeReset(self):
        """Initialize/reset learning timing variables.

        Called between episodes in `getting_started.py`.

        """
        # Timing stats variables.
        self.interstep_learning_time = 0
        self.interstep_learning_occurrences = 0
        self.interepisode_learning_time = 0