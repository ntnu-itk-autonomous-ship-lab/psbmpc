# PSB-MPC
<p>This repository implements the Probabilistic Scenario-based MPC in C/C++. The algorithm is an extended and improved version of the original one posed in [[1]](#1). A version is implemented using the CPU and another using the GPU (not finished yet!).  There are also some experimental setup for allowing obstacles to have their own collision avoidance setup, but this is not complete yet.<br>

Several test functions exist to showcase that the different library modules work as intented, and can be used for debugging or to make yourself familiar with the library.<br>

To use the library, for cmake, simply use the "add_subdirectory(/path/to/psbmpc_lib)" command, and link the corresponding target to your executable test file. </p>

## Dependencies

- Matlab is used for the debugging and plotting functionality.
- Eigen is used for the CPU and GPU version, with the 3.3.92 release used.
- CUDA and Thrust is necessary for the GPU version.

## Overall Structure
The library for the CPU-implementation has the following structure

├── psbmpc_lib
│   ├── CMakeLists.txt
│   │   ├── include
│   │   │   ├── cpe.h
│   │   │   ├── kf.h
│   │   │   ├── mrou.h
│   │   │   ├── obstacle.h
│   │   │   ├── obstacle_sbmpc.h
│   │   │   ├── obstacle_ship.h
│   │   │   ├── ownship.h
│   │   │   ├── prediction_obstacle.h
│   │   │   ├── psbmpc.h
│   │   │   ├── psbmpc_index.h
│   │   │   ├── tracked_obstacle.h
│   │   │   └── utilities.h
│   │   └── src
│   │       ├── cpe.cpp
│   │       ├── kf.cpp
│   │       ├── mrou.cpp
│   │       ├── obstacle.cpp
│   │       ├── obstacle_sbmpc.cpp
│   │       ├── obstacle_ship.cpp
│   │       ├── ownship.cpp
│   │       ├── prediction_obstacle.cpp
│   │       ├── psbmpc.cpp
│   │       ├── tracked_obstacle.cpp
│   │       └── utilities.cpp

with an explanation of the modules (classes) below 

### PSBMPC

<p>Main class of the library, implements the collision avoidance algorithm. Tuning is until now done in its constructor, but one can implement a read function to easily upload tuning changes via other files.<br>

The main function to use is the ***calculate_optimal_offsets(..)*** function, which requires the following ***inputs***:</p>

- Planned guidance references for surge and course
- The waypoints that the own-ship are planned to follow
- The current own-ship state (3DOF)
- Nearby obstacle states, an aggregated matrix with columns of [x, y, Vx, Vy, A, B, C, D, ID]^T where the first 4 variables are the north and east position and velocity, respectively. The A, B, C, D parameters are for a square approximation of the obstacle's dimensions, and ID is its indentification number.
- The corresponding covariance information or uncertainty associated with the estimates/measurement on [x, y, Vx, Vy]^T, flattened into a 16-element vector. 
- The corresponding intention probabilities for the obstacle, obtained by some intention inference module. If the PSB-MPC is configured to not consider intentions, these inputs are not used.
- The corresponding a priori probability of the obstacle being COLREGS compliant, obtained by some intention inference module. If the PSB-MPC is configured to not consider intentions, these inputs are not used.
- Nearby static obstacles, parameterized as for instance polygons, lines or similar. Not fully specified yet.

and has the following ***outputs***:

- Optimal surge and course modification to the planned guidance references
- A predicted trajectory for the own-ship whene implementing the optimal avoidance maneuver(s).
- A status matrix on the obstacles, displaying different information.
- A status vector on the collision avoidance system, displaying the minimal cost output associated with the optimal avoidance maneuver, and an ad hoc measure of its control freedom. 


### Obstacle

The obstacle class maintains information about the obstacle, in addition to its predicted trajectories and PSB-MPC cost function related parameters. Organized into a inheritance hierarchy with

- Obstacle : Base class holding general information
	- Tracked_Obstacle : Holding tracking and prediction related information and modules. This is the object maintained by the PSB-MPC to keep track of the nearby obstacles. 
	- Prediction_Obstacle: **Not to be used yet.** More minimalistic derived class than the Tracked_Obstacle, intended for use by obstacles in the PSB-MPC prediction when they have enabled their own collision avoidance system
	- (Cuda_Obstacle: For the GPU version we also have a tailor made derived class for use in the CUDA kernels, which needs special care.)

### Obstacle_Ship 

<p> This module implements a minimal kinematic module for the motion of a nearby obstacle with guidance and control, for use in the Obstacle SB-MPC predictions when the PSB-MPC enables obstacles to have their own collision avoidance system. The guidance is based on using the Speed over Ground (SOG) and Course over Ground (COG) for the obstacle directly, with some first order time constant delay.
The model is on the form <br>

x_{k+1} = x_{k} + U $\cos$($\chi_{k}$)
y_{k+1} = y_{k} + U $\sin$($\chi_{k}$)
$\chi$_{k+1} = $\chi_{k}$ + $\frac{1}{T_{\chi}}$($\chi_d$ - $\chi_{k}$)
U_{k+1} = U_{k} + $\frac{1}{T_{U}}$(U_d - U_{k})

Not tested yet. </p>

### Obstacle_SBMPC

<p>A simple SB-MPC meant for use by obstacles in the PSB-MPC prediction when considering intelligent obstacles. Not tested nor finished yet, so should not be used. </p>

### Ownship

<p>This module implements a 3DOF surface vessel model with guidance and control as used in for instance <https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2625756>. Should naturally match the underlying vessel. </p>


### KF

<p>This is a linear Kalman-filter module used when AIS-messages are received on obstacle positions, to enable more robustness for the PSB-MPC against track loss.  </p>

### MROU

<p>This is the Mean-reverting Ornstein-Uhlenbeck process used for the prediction of the independent obstacle trajectories and corresponding covariance. </p>

### CPE

<p>This is the Collision Probability Estimator used in the PSB-MPC predictions. Has incorporated two methods, one based on the Cross-Entropy method for estimation, and another based on [[2]](#2). The estimator is sampling-based, and is basically among others the main reason for trying to implement the PSB-MPC on the GPU. Estimates the probability with pairs of trajectories (of the own-ship and a nearby obstacle) as inputs.  

### Utilities

<p> Functions commonly used across multiple modules, gathered in one file. </p>


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." in 2020 23rd International Conference on Information Fusion (FUSION), South Africa, in press.


<p>*Written by Trym Tengesdal on 10. august 2020.*<br>
**
 </p>
