# PSB-MPC C++
<p> This repository implements libraries for the Probabilistic Scenario-based MPC [[2]](#2) in C/C++, one version for the CPU and another for the GPU (much faster). The algorithm is a new extended and improved version of the original one posed in [[1]](#1), which was implemented by Inger Hagen and Giorgio Kufoalor through the Autosea project (with added robustness against obstacle track loss etc. [[3]](#3). The libraries are located under libs/. </p>

## Dependencies

- Matlab C API for the debugging and plotting functionality. (Follow instructions at <https://www.mathworks.com/help/matlab/matlab_external/overview.html>)
- Eigen >= 3.3.7 (already included in repo under libs/third_party_libs/). Eigen is still experimental regarding CUDA compatibility. I have not suppressed the warnings from eigen regarding CUDA-stuff, but hope that one day Eigen will be fully functionable on the GPU. Not tested with other Eigen versions.
- xoshiro256+ random number generator used in the Collision Probability Estimator implemented for use in the CPU version (already included in repo under libs/third_party_libs/, implementation taken from <https://gist.github.com/imneme/3eb1bcc5418c4ae83c4c6a86d9cbb1cd#comments>). See <http://prng.di.unimi.it/> for more information. 
- CUDA and Thrust for the GPU version. Not tested for CUDA versions below 10.0.
- cuRAND <https://docs.nvidia.com/cuda/curand/index.html> is used for the Collision Probability Estimator compatible on the device. 

## PSB-MPC library usage
<p>To use the library, for cmake, simply use the "add_subdirectory(/path/to/psbmpc_lib)" command, and link the corresponding target to your executable test file. Then, change directory to either debug or release, and build using standard cmake commands for either a debug or release build. </p>

## Testing out the library functionality
<p> Several test functions exist under src/tests to showcase that the different library modules work as intented, and can be used for debugging or to make yourself familiar with the library. By opening the CMakeLists.txt files one can specify the module one wish to test. </p>


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." 2020 23rd International Conference on Information Fusion (FUSION), South Africa.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).



<p> Trym Tengesdal, 17. March 2021.  </p>
