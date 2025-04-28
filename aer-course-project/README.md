# AER1217_Fianl_Project
# Autuonomoud Drone Racing Competition

Overview

This project implements an autonomous drone racing system that uses gate and obstacle positions from getting_started.yaml to plan and execute a collision-free trajectory. A 2D occupancy grid at z = 1.0m is constructed, with obstacles modeled as 0.5m exclusion zones and gates as straw-shaped aligned regions. An A* algorithm plans a path through the gates using Euclidean distance and 0.1m grid resolution. The path is smoothed with B-spline interpolation and converted into a sequence of full-state waypoints. The drone follows the planned path using precise control commands, and uses simple takeoff and landing commands to start and end the flight with the Crazyfile. A feed-forward controller monitors position feedback via adjusting motion when deviation is detected. The system was validated in both simulation and real-world trials, demonstrating stable and accurate gate traversal.

Project Structure

Requirements:

Ensure you have the following dependencies installed:

Dependencies:
Numpy
PyBullet
Matplotlib
Safe-control-gym
Pycffirmware

Usage:

Place all project files and getting_started.yaml in the aer-course-project folder of Safe-Control-Gym. If using firmware, build pycffirmware using build_linux.sh. Run the simulation by executing final_project.py with --overrides ./getting_started.yaml to start the drone racing task in PyBullet.

Run the simulation script:

The script will read gate and obstacle positions from getting_started.yaml, plan a trajectory using a velocity-aware RRT algorithm, and smooth the path using B-spline interpolation. It will then execute the trajectory in PyBullet using Crazyflie control commands. Outputs include a visual simulation of the drone navigating through gates and performance plots showing speed and trajectory deviation.

Functionality:

Path Planning:
Reads gate and obstacle data from getting_started.yaml.
Generates a waypoint path using the A* algorithm on a 2D occupancy grid at z = 1.0m.
Uses Euclidean distance as the heuristic and a 0.1m node resolution for efficiency.

Obstacle and Gate Modeling:
Models obstacles as 0.5m square exclusion zones.
Represents gates using two rectangular keep-out zones aligned in the direction of flight.
Ensures proper alignment during gate traversal to avoid clipping.

Trajectory Smoothing:
Applies B-spline interpolation to convert the discrete A* path into a smooth, flyable trajectory.
Outputs a sequence of waypoints for control execution.

Output:
Displays a PyBullet simulation of the drone flying through the gate course.
Generates velocity and deviation plots to visualize flight performance.
