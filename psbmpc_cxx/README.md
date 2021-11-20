# PSB-MPC 1.0 C++
This repository implements a library for the Probabilistic Scenario-based MPC [[2]](#2) in C++ and CUDA, one version for the CPU and another for the GPU (much faster). The algorithm is a new extended and improved version of the original one posed in [[1]](#1), which was implemented by Inger Hagen and Giorgio Kufoalor through the Autosea project (with added robustness against obstacle track loss etc. [[3]](#3). The library is located under libs.

The library heavily relies on cmake, which you can learn more about e.g. here: <https://cliutils.gitlab.io/modern-cmake/> 

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

## Building the library
Before you build and install the library locally in your computer, decide on the following:

- If you want to build a debug (set compile time flag `CMAKE_BUILD_TYPE=debug`) or release (set flag to `CMAKE_BUILD_TYPE=release`) version.
- If you want to use the GPU-functionality (set `USE_GPU_PSBMPC=1`, the default is 0).
- If you want to enable Matlab and Debugging functionality (set `ENABLE_PSBMPC_DEBUGGING=1`, the default is 0).
- What own-ship type you want to use as model in the MPC (set `OWNSHIP_TYPE=0` : Kinematic model, `OWNSHIP_TYPE=1` : Kinetic model). More details about this variable under **psbmpc_lib**

Then, after figuring out this, you build and install the library with the following commands (assuming you are in the root directory **psbmpc_cxx/**:
- 1: `mkdir build_type && cd build_type`
- 2: `cmake -DCMAKE_BUILD_TYPE=build_type -DUSE_GPU_PSBMPC=use_gpu_psbmpc -DENABLE_PSBMPC_DEBUGGING=enable_psbmpc_debugging -DOWNSHIP_TYPE=ownship_type ..`
- 3: `sudo make install -j4`

where `build_type`, `use_gpu_psbmpc`, `enable_psbmpc_debugging` and `ownship_type` are chosen by you. The library is now installed with headers to `usr/local/include/`, static library file to `usr/local/lib/` and cmake config files to `usr/local/share/`. 

You can now use the library in your project with `find_package(PSBMPC1 REQUIRED)`, and link to the PSBMPC-target using `target_link_libraries(your_executable_target PSBMPC1::PSBMPC)`.

## Getting familiar with the library
Several test functions exist under tests for testing that the different library modules work as intented, and can be used for debugging or to make yourself familiar with the library. By opening the CMakeLists.txt files one can specify the module one wish to test, by using the `add_executable(..)` command. To use the library with one of the test files under tests, for cmake, specify your chosen compile time flags, go into the chosen directory and build using 

- 1: `mkdir build_type && cd build_type`
- 2: `cmake -DCMAKE_BUILD_TYPE=build_type -DUSE_GPU_PSBMPC=use_gpu_psbmpc -DENABLE_PSBMPC_DEBUGGING=enable_psbmpc_debugging -DOWNSHIP_TYPE=ownship_type ..`
- 3: `make`

The generated executable file is then located in the build folder, which you can run using `./tester` in the terminal.

## Common Pitfalls

- Include error of some kind during compilation or when using your IDE: Re-check that you have the required dependencies (especially for Matlab, Eigen3 and Boost) and that they are included properly. 
- Matlab engine start failed when attempting to run one of the test files. The most common error is that you did not add the MATLAB path to the environmental variables properly, as e.g. <https://se.mathworks.com/help/matlab/matlab_external/set-run-time-library-path-on-linux-systems.html> for Linux. 
- Wrong configuration of the cmake flags (`USE_GPU_PSBMPC` etc..)


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." 2020 23rd International Conference on Information Fusion (FUSION), South Africa.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).

<a id="4">[4]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2021). "Ship Collision Avoidance Utilizing the Cross-Entropy Method for Collision Risk Assessment." IEEE Transactions on Intelligent Transportation Systems, p. 1-14.


<p> Trym Tengesdal, 20. November 2021.  </p>
