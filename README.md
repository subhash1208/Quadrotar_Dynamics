# Quadrotar Dynamics
This repository contains code for simulating the dynamics of a quadrotar and generating optimal trajectories for it.

## Dependencies

* MATLAB (version 2019a or later)
* Simulink (version 2019a or later)
* MATLAB Libraries:
	+ Control Systems Toolbox
	+ Aerospace Toolbox
	+ Optimization Toolbox
	+ Symbolic Math Toolbox


## PD Controller

- Run code: change trajectories in file `control/runsim.m` and run.
- See `control/controller.m` for implementation of the PD controller.

### Code Explanation

The PD controller code is organized into the following MATLAB files:

`control/controller.m`: This file implements the PD controller, which takes in the desired trajectory and the current state of the quadrotor, and outputs the control inputs (rotor speeds) to track the desired trajectory.

`control/runsim.m`: This file runs the simulation of the PD controller, using the desired trajectory and initial conditions specified in the file.

`control/pd_gain_tuning.m`: This file contains code for tuning the PD controller gains to achieve stable and responsive tracking of the desired trajectory.

To run the PD controller simulation, simply execute the `control/runsim.m` file in MATLAB. The script will simulate the dynamics of the quadrotor using the PD controller and display the results in a plot.

## Path Planning and Trajectory Optimization

- Run code: `traj_planning/runsim.m` and run path 1 or path 3.
- See `traj_planning/path_planning/dijkstra.m` for implementation of path finding algorithms (dijstra, A*).
- See `traj_planning/traj_opt7.m` for implementations of minimium snap trajectory.
- See `traj_planning/traj_opt5.m` for implementations of minimium acceleration trajectory.

### Code Explanation

The path planning and trajectory optimization code is organized into the following MATLAB files:

`traj_planning/runsim.m`: This file runs the simulation of the path planning and trajectory optimization algorithms, using the start and end points specified in the file.

`traj_planning/path_planning/dijkstra.m`: This file implements Dijkstra's algorithm for path finding, which finds the shortest path between the start and end points.

`traj_planning/traj_opt7.m`: This file implements the minimum snap trajectory optimization algorithm, which generates a smooth and feasible trajectory that minimizes the snap (jerk) of the quadrotor.

`traj_planning/traj_opt5.m`: This file implements the minimum acceleration trajectory optimization algorithm, which generates a smooth and feasible trajectory that minimizes the acceleration of the quadrotor.

To run the path planning and trajectory optimization simulation, simply execute the `traj_planning/runsim.m` file in MATLAB. The script will simulate the path planning and trajectory optimization algorithms and display the results in a plot.

### Visualization

Visualization below. Desired (blue) vs Actual (red)

#### Trajectory 1: Step

<img src="gifs/p1p1_step.gif" alt="step" width="270"> <img src="imgs/p1p1_step_p.jpg" alt="step" width="270"> <img src="imgs/p1p1_step_v.jpg" alt="step" width="270">

#### Trajectory 2: Circle

<img src="gifs/p1p1_circle.gif" alt="step" width="270"> <img src="imgs/p1p1_circle_p.jpg" alt="step" width="270"> <img src="imgs/p1p1_circle_v.jpg" alt="step" width="270">

#### Trajectory 2: Diamond

<img src="gifs/p1p1_diamond.gif" alt="step" width="270"> <img src="imgs/p1p1_diamond_p.jpg" alt="step" width="270"> <img src="imgs/p1p1_diamond_v.jpg" alt="step" width="270">

#### Minimum Acceleration Trajectory

<img src="gifs/p1p3_map1_acc.gif" alt="step" width="270"> <img src="imgs/p1p3_map1_acc_p.jpg" alt="step" width="270"> <img src="imgs/p1p3_map1_acc_v.jpg" alt="step" width="270">

<img src="gifs/p1p3_map3_mini_acc.gif" alt="step" width="270"> <img src="imgs/p1p3_map3_mini_acc_p.jpg" alt="step" width="270"> <img src="imgs/p1p3_map3_mini_acc_v.jpg" alt="step" width="270">

#### Minimum Snap Trajectory

<img src="gifs/p1p3_map1_snap.gif" alt="step" width="270"> <img src="imgs/p1p3_map1_snap_p.jpg" alt="step" width="270"> <img src="imgs/p1p3_map1_snap_v.jpg" alt="step" width="270">



Please note that this implementation assumes the quadrotor dynamics are modeled accurately, and there may be differences between the simulation results and actual physical behavior. For real-world applications, additional considerations may be necessary, such as modeling nonlinearities and actuator constraints.
```
Please let me know if you have any further questions or concerns.
```