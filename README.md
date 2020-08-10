# PSB-MPC
<p> This repository implements the Probabilistic Scenario-based MPC in C/C++. The algorithm is an extended and improved version of the original one posed in [[1]](#1). A version is implemented using the CPU and another using the GPU (not finished yet!). There are also some experimental setup for allowing obstacles to have their own collision avoidance setup, but this is not complete yet.<br>

Several test functions exist under src/tests to showcase that the different library modules work as intented, and can be used for debugging or to make yourself familiar with the library. By opening the CMakeLists.txt files one can specify the module one wish to test.<br>

To use the library, for cmake, simply use the "add_subdirectory(/path/to/psbmpc_lib)" command, and link the corresponding target to your executable test file. The libraries are located under /libs/. </p>

## Dependencies

- Matlab C API is used for the debugging and plotting functionality.
- Eigen 3.3.92 release is used for the CPU and GPU version.
- CUDA and Thrust is necessary for the GPU version.

## Overall Structure
The library for the CPU-implementation has the following structure

<img src="tree_psbmpc.png" width="400"> 

with an explanation of the modules (classes) below: 

### PSBMPC

<p> Main class of the library, implements the collision avoidance algorithm. Tuning is until now done in its constructor, but one can implement a read function to easily upload tuning changes via other files.<br>

The main function to use is the **calculate_optimal_offsets(..)** function, which requires the following **inputs**: </p>

- Planned guidance references for surge and course
- The waypoints that the own-ship is supposed to follow (curved/continuous path following is not implemented)
- The current own-ship state (3DOF) <img src="https://render.githubusercontent.com/render/math?math={[x, y, \psi, u, v, r]}^T">
- Nearby obstacle states, an aggregated matrix with columns of <img src="https://render.githubusercontent.com/render/math?math={[x, y, V_x, V_y, A, B, C, D, ID]}^T"> where the first 4 variables are the north and east position and velocity, respectively. The A, B, C, D parameters are the square approximation of the obstacle's dimensions, and ID is its indentification number.
- The corresponding covariance information or uncertainty associated with the estimates/measurement on <img src="https://render.githubusercontent.com/render/math?math={[x, y, V_x, V_y]}^T">, flattened into a 16-element vector. 
- The corresponding intention probabilities for the obstacle, obtained by some intention inference module. If the PSB-MPC is configured to not consider intentions, these inputs are not used.
- The corresponding a priori probability of the obstacle being COLREGS compliant, obtained by some intention inference module. If the PSB-MPC is configured to not consider intentions, these inputs are not used.
- Nearby static obstacles, parameterized as for instance polygons, lines or similar. Not fully specified yet.

and has the following **outputs**:

- Optimal surge and course modification to the planned guidance references
- A predicted trajectory for the own-ship when implementing the optimal avoidance maneuver(s).
- A status matrix on the obstacles, displaying different information.
- A status vector on the collision avoidance system, displaying the minimal cost output associated with the optimal avoidance maneuver, and an ad hoc measure of its control freedom. 


### Obstacle

The obstacle class maintains information about the obstacle, in addition to its predicted trajectories and PSB-MPC cost function related parameters. Organized into a inheritance hierarchy with

- Obstacle : Base class holding general information
	- Tracked_Obstacle : Holding tracking and prediction related information and modules. This is the object maintained by the PSB-MPC to keep track of the nearby obstacles. 
	- Prediction_Obstacle: **Not to be used yet**. More minimalistic derived class than the Tracked_Obstacle, intended for use by obstacles in the PSB-MPC prediction when they have enabled their own collision avoidance system
	- (Cuda_Obstacle: For the GPU version we also have a tailor made derived class for use in the CUDA kernels, which needs special care. **Not complete yet**)

### Obstacle_Ship 

<p> This module implements a minimal kinematic module for the motion of a nearby obstacle with guidance and control, for use in the Obstacle SB-MPC predictions when the PSB-MPC enables obstacles to have their own collision avoidance system. The guidance is based on using the Speed over Ground (SOG) and Course over Ground (COG) for the obstacle directly, with some first order time constant delay.
The model is on the form <br>

<img src="https://render.githubusercontent.com/render/math?math=x_{k%2B1} = x_{k} %2B U_k \cos(\chi_{k})"> <br>
<img src="https://render.githubusercontent.com/render/math?math=y_{k%2B1} = y_{k} %2B U_k \sin(\chi_{k})"> <br>
<img src="https://render.githubusercontent.com/render/math?math=\chi_{k%2B1} = \chi_{k} %2B \frac{1}{T_{\chi}}(\chi_d - \chi_{k})"> <br>
<img src="https://render.githubusercontent.com/render/math?math=U_{k%2B1} = U_{k} %2B \frac{1}{T_{U}}(U_d - U_{k})"> <br>

**Not fully tested** yet. </p>

### Obstacle_SBMPC

A simple SB-MPC meant for use by obstacles in the PSB-MPC prediction when considering intelligent obstacles. **Not tested nor finished yet**, so should not be used. 

### Ownship

This module implements a 3DOF surface vessel model with guidance and control as used in for instance <https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2625756>. Should naturally match the underlying vessel.


### KF

This is a linear Kalman-filter module used when AIS-messages are received on obstacle positions, speed and course, to enable more robustness for the PSB-MPC against track loss and missing AIS-messages. 

### MROU

This is the Mean-reverting Ornstein-Uhlenbeck process used for the prediction of the independent obstacle trajectories and corresponding covariance. 

### CPE

This is the Collision Probability Estimator used in the PSB-MPC predictions. Has incorporated two methods, one based on the Cross-Entropy method for estimation (reference will be underway soon enough), and another based on [[2]](#2). The estimator is sampling-based, and is basically among others the main reason for trying to implement the PSB-MPC on the GPU. Estimates the probability with pairs of trajectories (of the own-ship and a nearby obstacle) as inputs. 

### Utilities

<p> Inlined functions commonly used across multiple modules, gathered in one file. </p>


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." in 2020 23rd International Conference on Information Fusion (FUSION), South Africa, in press.



<p> _Written by Trym Tengesdal on 10. august 2020._  </p>
