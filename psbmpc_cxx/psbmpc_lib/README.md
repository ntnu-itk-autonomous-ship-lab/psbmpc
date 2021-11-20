# PSB-MPC 1.0 Library
This is the Probabilistic Scenario-based MPC in C/C++, which facilitates a CPU and a GPU version of the algorithm. For the GPU version, the most time-critical part of the MPC is implemented in CUDA to allow for performance gains through parallelization. 

To use the library, for cmake, simply use the `add_subdirectory(/path/to/psbmpc_lib)` command, and link the corresponding target to your executable test file. Then, change directory to either debug or release, and build using standard cmake commands for either a debug or release build.

Note that the amount of memory you need on your GPU to run the algorithm will increase alot with the number of maneuvers (and the set of different maneuver types) you consider in the prediction horizon. The **psbmpc_defines.h** header should be used to set limits to maximum number of maneuvers, obstacles etc. to reflect the memory constraint on your computer. The library has mainly been tested with GeForce RTX 3090 with 24 GB memory. For GPUs with less than or equal to 8 GB memory, test with 1 maneuver first, and gradually increase until you hit the memory constraint. This is considering this specific implementation, and it can be optimized wrt memory, although it would most likely come at the cost of degraded modularity in the cuda code.


## Dependencies

- C++17
- CMake > 3.17 for building 
- Matlab C API for the debugging and plotting functionality. (Follow setup instructions at <https://www.mathworks.com/help/matlab/matlab_external/overview.html>)
- Eigen3. Eigen is still experimental regarding CUDA compatibility. I have suppressed the warnings from eigen regarding CUDA-stuff, but hope that one day Eigen will be fully functionable and warning-free on the GPU. Not tested with other Eigen versions. On Linux it can be installed using `sudo apt install libeigen3-dev`
- xoshiro256+ random number generator used in the Collision Probability Estimator implemented for use in the CPU version (already included in the psbmpc library, implementation taken from <https://gist.github.com/imneme/3eb1bcc5418c4ae83c4c6a86d9cbb1cd#comments>). See <http://prng.di.unimi.it/> for more information. 
- CUDA and Thrust for the GPU version. Not tested for CUDA versions below 10.0. Install the CUDA toolkit at <https://developer.nvidia.com/cuda-toolkit>, and follow install instructions at <https://docs.nvidia.com/cuda/index.html>
- cuRAND <https://docs.nvidia.com/cuda/curand/index.html> is used for the Collision Probability Estimator compatible on the device. Already included through CUDA. 
- Boost <https://www.boost.org/> for reading shapefile data into a vector of polygons in the Grounding Hazard Manager, and used in the grounding cost calculation of the PSB-MPC CPU version. On Linux it can be installed using `sudo apt-get install libboost-all-dev`
- GeographicLib <https://geographiclib.sourceforge.io/html/index.html> for conversion between different coordinate types (UTM, UPS; cartesian, geocentric etc). Follow install instructions at the webpage.

## Overall Structure
The library has a main library namespace **PSBMPC_LIB** which contains all the functionality. The namespace is further nested into **CPU** and **GPU** for versions of classes (with the same name, e.g. PSBMPC, Ownship,..) that have different implementation for the CPU and GPU version of the MPC, respectively. Common classes/functionality exists under the library namespace, such as a Kalman Filter, SBMPC, Mean-Reverting Ornstein-Uhlenbeck process, etc. 

As Matlab is used for debugging and visualization, there are sections of commented out code in e.g. psbmpc.cpp that has the "title" "MATLAB PLOTTING FOR DEBUGGING". This code can be commented in to use to plot the different cost terms in the MPC, visualize the environment, plot the current own-ship trajectory, predicted obstacle trajectories, and plot the collision probabilities associated with them. Note, you have to build the library with `ENABLE_PSBMPC_DEBUGGING=1` to use this functionality, and have Matlab installed and configured.

The main modules (classes/structs) are explained below:

### PSBMPC

Main class of the library, implements the collision avoidance algorithm.

The main function to use is the **calculate_optimal_offsets(..)** function, which requires the following **inputs**:

- Planned guidance references for surge and course
- The waypoints that the own-ship is supposed to follow (curved/continuous path following is not implemented yet)
- The current time own-ship state (3DOF) <img src="https://render.githubusercontent.com/render/math?math={[x, y, \psi, u, v, r]}^T">
- Absolute wind speed and its direction (used with the anti-grounding part)
- Nearby static obstacles, parameterized as polygons or no-go lines.
- A data structure Obstacle Data containing dynamic obstacle information.

and has the following **outputs**:

- Optimal surge and course modification to the planned guidance references
- A predicted trajectory for the own-ship when implementing the optimal avoidance maneuver(s).
- Obstacle Data: Some parts of the Obstacle Data can be modified by the PSB-MPC (predicted relative hazard levels for each obstacle)

You should use the Obstacle Manager class for creating and updating the Obstacle Data structure, and can also use the Grounding Hazard Manager to create and update relevant polygons for the anti-grounding part. The Obstacle Predictor class is used to predict all nearby dynamic obstacle trajectories (can be multiple trajectories for each obstacle), using the Obstacle Data from the Obstacle Manager as input. 

### PSBMPC Parameters
Contains all PSB-MPC parameters in a class, which should be modified according to tuning changes. The class has get/set functionality for each parameter according to an index file **psbmpc_index.h**, and uses limits on double and integer type parameters to assure that the setting of these parameters makes sense. Work could although be done to make the get/set functionality even better.

Note that the amount of control behaviours (function of the amount of maneuvers and different maneuver types considered) that can be considered on the GPU, is highly limited by the amount of memory available on the GPU, as some data structures (for instance the CPE class) need to be allocated and transferred to each thread (host to device transfer). 

### SBMPC
The original SBMPC, made into a separate class for the library to enable easier comparison.

### SBMPC Parameters 
Parameter class for the SBMPC.

### MPC Cost
Class responsible for evaluating the cost function in the PSBMPC, SBMPC and obstacle SBMPC. One version each, meant for the host/device side.

## CB Cost Functor (1 & 2)
Special case C++ class which has overloaded the **operator(..)**. The functors 1 and 2 are used to evaluate the cost of following one particular control behaviour. The functors are ported to the gpu, where each thread will run the **operator(..)** to evaluate the cost of a certain control behaviour. The first **CB_Cost_Functor_1** predicts the own-ship trajectory for all control behaviours, and calculates the path related costs. After this, the second **CB_Cost_Functor_2** calculates the cost for one control behaviour wrt one static obstacle OR a dynamic obstacle behaving as in a certain prediction scenario. The total cost of each control behaviour is then stitched together on the host side in the **find_optimal_control_behaviour()** function.

### CB Cost Functor Structures 
 Defines data for GPU threads that is needed in the **CB_Cost_Functor**, which needs to be sent from the host to the device. A subset of the PSB-MPC parameters are defined in a struct here, and also a struct which gathers diverse types of data for use on the GPU. 

### Grounding Hazard Manager
Reads in ENC data of land in the form of shapefiles into 2D polygons. Simplifies the polygons using the Ramer-Douglas-Peucker algorithm <https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm>, and returns a reference to the relevant ones for usage in the PSBMPC. Has a UTM Projection class to manage conversions between latitude longitude and UTM Zone Projection coordinates.

### Obstacle Predictor
Responsible for setting up and predicting the dynamic obstacle trajectories, used by the PSB-MPC. Takes in and updates an Obstacle Data structure with the predicted trajectory information.

### Obstacle Manager

Is the class responsible for updating dynamic obstacle information, taking the following inputs in its main update functionality:

- Nearby obstacle states, an aggregated matrix with columns of <img src="https://render.githubusercontent.com/render/math?math={[x, y, V_x, V_y, A, B, C, D, ID]}^T"> where the first 4 variables are the north and east position and velocity, respectively. The A, B, C, D parameters are the square approximation of the obstacle's dimensions, and ID is its indentification number.
- The corresponding covariance information or uncertainty associated with the estimates/measurement on <img src="https://render.githubusercontent.com/render/math?math={[x, y, V_x, V_y]}^T">, flattened into a 16-element vector. 

and also updates the current situation type that the own-ship is in, wrt to each obstacle, and also transitional variables (if an obstacle is passed by, is head on, is ahead, is overtaking the own-ship etc.).

An Obstacle_Data structure containing this dynamic obstacle information is sent to the PSBMPC at each COLAV iteration.

### Joint Prediction Manager

Prototype class, similar to the Obstacle Manager, just that it keeps track of information for all Prediction_Obstacles in the PSBMPC prediction where obstacles are assumed to have their own COLAV system. Only for the CPU-PSB-MPC, but not currently in use because joint prediction is too slow, so **Not used nor maintained**

### Obstacle Types

The obstacle classes maintains information about the obstacle, in addition to its predicted trajectories and PSB-MPC cost function related parameters. Organized into a inheritance hierarchy with 

- Tracked Obstacle : Holding tracking and prediction related information and modules. This is the object maintained by the PSB-MPC to keep track of the nearby obstacles. 
- Prediction Obstacle: More minimalistic class than the Tracked Obstacle, used by obstacles in the PSB-MPC prediction when they have enabled their own collision avoidance system. **Not used nor maintained**
- Cuda Obstacle: Used as a GPU-friendly data container of relevant Tracked Obstacle data needed on the GPU. Read-only when processing on the GPU.


### Obstacle SBMPC Parameters

Special case parameter class for the Obstacle SBMPC which is used on the GPU. Uses the TML library instead of Eigen for matrix/vector types.

### Obstacle SBMPC

A simple SB-MPC meant for use by obstacles in the PSB-MPC prediction when considering intelligent obstacles. One version each for the CPU/GPU implementation.

### Kinetic Ship Models

Implements a 3DOF surface vessel base model class with guidance and control as used in for instance <https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2625756>. One version each for the CPU/GPU implementation. By specifying the compile time flag **OWNSHIP_TYPE**, one can choose between the derived versions Telemetron and MilliAmpere(**NOT FINISHED**).

### Kinematic Ship Models

This module implements a minimal kinematic module for the motion of a nearby obstacle with guidance and control, for use in the Obstacle SB-MPC predictions when the PSB-MPC enables obstacles to have their own collision avoidance system. The guidance is based on using the Speed over Ground (SOG) and Course over Ground (COG) for the obstacle directly, with first order time constant delay.
The model is on the form

<img src="https://render.githubusercontent.com/render/math?math=x_{k%2B1} = x_{k} %2B \Delta_t U_k \cos(\chi_{k})">
<img src="https://render.githubusercontent.com/render/math?math=y_{k%2B1} = y_{k} %2B \Delta_t U_k \sin(\chi_{k})"> 
<img src="https://render.githubusercontent.com/render/math?math=\chi_{k%2B1} = \chi_{k} %2B \Delta_t \frac{1}{T_{\chi}}(\chi_d - \chi_{k})">
<img src="https://render.githubusercontent.com/render/math?math=U_{k%2B1} = U_{k} %2B \Delta_t \frac{1}{T_{U}}(U_d - U_{k})">

 One version each for the CPU/GPU implementation. By specifying the compile time flag **OWNSHIP_TYPE**, one can choose to use this model for the Ownship. This kinematic model is the default for the Obstacle Ship.  

### KF

This is a linear Kalman-filter module used when obstacle states are received from a tracker node, to enable more robustness for the PSB-MPC against track loss [[3]](#3). 

### MROU

This is the Mean-reverting Ornstein-Uhlenbeck process used for the prediction of the independent obstacle trajectories and covariance. 

### CPE

This is the Collision Probability Estimator used in the PSB-MPC predictions. Has incorporated two methods, one based on the Cross-Entropy method [[4]](#4) for estimation (reference will be underway soon enough), and another based on [[2]](#2). The estimator is sampling-based, and is among others the main reason for trying to implement the PSB-MPC on the GPU. **NOTE** Changed to facilitate only static data allocation, and only considers one obstacle at the time. A grid of CPEs is allocated prior to running GPU code, where each thread will read/write to their own CPE object. One version each for the CPU/GPU implementation, as the GPU version requires a tailor made PRNG, whereas the CPU version can use std:: type or other custom PRNG (like xoshiro, which is fast and efficient). 

### Utilities

 Inlined functions commonly used across multiple modules, gathered in one file. One version each for the CPU/GPU implementation. 


## TML (Tryms (shitty) matrix library)
Custom matrix library made specifically for usage of matrices in CUDA kernels, as I did not find another satisfactory third-party solution for this. Hopefully, Eigen will have better CUDA support in the future, which is unfortunately very limited today. **NOTE:** This library should be used with care, as it is only tested for a subset of all "typical matrix functionality", i.e. only the operations currently used in the PSB-MPC GPU run code. 

The library implements three matrix type containers: 
- Static Matrix: Fixed sized matrices
- Pseudo Dynamic Matrix (PDMatrix): (Fixed size) Matrix used to store larger amounts of data, with a compile-time known max number of rows and columns. However, the effective size used during run-time can vary.
- Dynamic Matrix: Matrix container for data with  varying size **Not used nor maintained**

Only the fixed size matrices are used currently, because dynamic memory allocation on the gpu is costly, slow and should therefore in general not be done. Thus, the *dynamic_matrix.cuh* file is **NOT USED**. 

## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." in 2020 23rd International Conference on Information Fusion (FUSION), South Africa, in press.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).

<a id="4">[4]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2021). "Ship Collision Avoidance Utilizing the Cross-Entropy Method for Collision Risk Assessment." IEEE Transactions on Intelligent Transportation Systems, p. 1-14.



 Trym Tengesdal, 20. November 2021.  
