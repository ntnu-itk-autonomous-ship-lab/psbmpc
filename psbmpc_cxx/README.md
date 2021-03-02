# PSB-MPC C++
<p> This repository implements libraries for the Probabilistic Scenario-based MPC [[2]](#2) in C/C++, one version for the CPU and another experimental for the GPU. The algorithm is a new extended and improved version of the original one posed in [[1]](#1), which was implemented by Inger Hagen and Giorgio Kufoalor through the Autosea project (with added robustness against obstacle track loss etc. [[3]](#3). The libraries are located under libs/. </p>

## Dependencies

- Matlab C API for the debugging and plotting functionality. (Follow instructions at <https://www.mathworks.com/help/matlab/matlab_external/overview.html>)
- Eigen 3.3.7 (Already included in repo under libs/third_party_libs/. Not tested with other versions)
- CUDA >= 10.0 and Thrust for the GPU stuff. Not tested for CUDA versions below 10.0.
- xoshiro256+ random number generator used in the Collision Probability Estimator implemented for use in the CPU version (already included in repo under libs/third_party_libs/, implementation taken from <https://gist.github.com/imneme/3eb1bcc5418c4ae83c4c6a86d9cbb1cd#comments>). See <http://prng.di.unimi.it/> for more information. 

## PSB-MPC library usage
<p>To use the library, for cmake, simply use the "add_subdirectory(/path/to/psbmpc_lib)" command, and link the corresponding target to your executable test file. Then, change directory to either debug or release, and build using standard cmake commands for either a debug or release build. </p>

## Testing out the library functionality
<p> Several test functions exist under src/tests to showcase that the different library modules work as intented, and can be used for debugging or to make yourself familiar with the library. By opening the CMakeLists.txt files one can specify the module one wish to test. NOTE: The .cpp test files are the best for use to make yourself familiar, whereas the .cu CUDA-based test files are experimental and cannot be guaranteed bug free at the moment. </p>


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." 2020 23rd International Conference on Information Fusion (FUSION), South Africa.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).



<p> Trym Tengesdal, 2. March 2021.  </p>
