# PSB-MPC C++
<p> This repository implements a library for the Probabilistic Scenario-based MPC [[2]](#2) in C++ and CUDA, one version for the CPU and another for the GPU (much faster). The algorithm is a new extended and improved version of the original one posed in [[1]](#1), which was implemented by Inger Hagen and Giorgio Kufoalor through the Autosea project (with added robustness against obstacle track loss etc. [[3]](#3). The library is located under libs. <br>

The library heavily relies on cmake, which you can learn more about e.g. here: <https://cliutils.gitlab.io/modern-cmake/> </p>

## Dependencies

- C++17
- CMake > 3.10 for building 
- Matlab C API for the debugging and plotting functionality. (Follow setup instructions at <https://www.mathworks.com/help/matlab/matlab_external/overview.html>)
- Eigen3. Eigen is still experimental regarding CUDA compatibility. I have suppressed the warnings from eigen regarding CUDA-stuff, but hope that one day Eigen will be fully functionable and warning-free on the GPU. Not tested with other Eigen versions.
- xoshiro256+ random number generator used in the Collision Probability Estimator implemented for use in the CPU version (already included in repo under *libs/third_party_libs/*, implementation taken from <https://gist.github.com/imneme/3eb1bcc5418c4ae83c4c6a86d9cbb1cd#comments>). See <http://prng.di.unimi.it/> for more information. 
- CUDA and Thrust for the GPU version. Not tested for CUDA versions below 10.0.
- cuRAND <https://docs.nvidia.com/cuda/curand/index.html> is used for the Collision Probability Estimator compatible on the device. 
- Boost <https://www.boost.org/> for reading shapefile data into a vector of polygons in the Grounding Hazard Manager, and used in the grounding cost calculation of the PSB-MPC CPU version.

## PSB-MPC library usage
<p> Several test functions exist under *src/tests* to showcase that the different library modules work as intented, and can be used for debugging or to make yourself familiar with the library. By opening the CMakeLists.txt files one can specify the module one wish to test, by using the `add_executable(..)` command. To use the library with one of the test files under *src/tests/*, for cmake, create a debug and or release directory, go into the chosen directory and type <br>  

`cmake -DCMAKE_BUILD_TYPE=debug ..` 

for debug and  <br>

`cmake -DCMAKE_BUILD_TYPE=release ..` 

for release. To use the GPU PSB-MPC, you also have to specify the `USE_GPU_PSBMPC=1` compile time flag: <br>

`cmake -DCMAKE_BUILD_TYPE=debug -DUSE_GPU_PSBMPC=1 ..` 

The same goes for toggling between own-ship types (e.g. `-DOWNSHIP_TYPE=0`). <br>

To use the library in another cmake project, copy the library folder into your project, and use the `add_subdirectory(/path/to/psbmpc_lib)` command to add it to your project. </p>

## Common Pitfalls

- Include error of some kind during compilation or when using your IDE: Re-check that you have the required dependencies (especially for Matlab, Eigen3 and Boost) and that they are included properly. 
- Matlab engine start failed when attempting to run one of the test files. The most common error is that you did not add the MATLAB path to the environmental variables properly, as e.g. <https://se.mathworks.com/help/matlab/matlab_external/set-run-time-library-path-on-linux-systems.html> for Linux. 


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." 2020 23rd International Conference on Information Fusion (FUSION), South Africa.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).



<p> Trym Tengesdal, 10. August 2021.  </p>
