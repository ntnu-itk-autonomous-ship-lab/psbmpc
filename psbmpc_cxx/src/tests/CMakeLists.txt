#======================================================================
# This CMakeLists.txt creates the target for use in testing PSB-MPC
# functionality. Every module of the PSB-MPC lib can be tested, by
# uncommenting the module test file you want to select. 

# NOTE: Not all the GPU type (.cu extensions) test files can be
# guaranteed to be bug-free, so use with care in this case. 

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

find_package(Eigen3 REQUIRED)
#TODO: Merge the gpu/cpu test files for gpu/cpu specific classes together

# Pure host side test files
add_executable(tester test_psbmpc.cpp)
#add_executable(tester test_ownship_cpu.cpp)
#add_executable(tester test_obstacle_ship_cpu.cpp)
#add_executable(tester test_obstacle_cpu.cpp)
#add_executable(tester test_obstacle_predictor.cpp)
#add_executable(tester test_cpe_cpu.cpp)
#add_executable(tester test_mrou.cpp)
#add_executable(tester test_kf.cpp)
#add_executable(tester test_prng.cpp)

#add_executable(tester test_psbmpc_grounding.cpp)
#add_executable(tester test_read_shapefile.cpp)
#add_executable(tester test_polygons.cpp)

# Test files specific to the GPU-version of the PSBMPC
#add_executable(tester test_ownship_gpu.cu)
#add_executable(tester test_obstacle_ship_gpu.cu)
#add_executable(tester test_cpe_gpu.cu)
#add_executable(tester test_thrust.cu)
#add_executable(tester test_tml.cu)
#add_executable(tester test_tml_pdmatrix.cu)

#add_executable(tester paper_simulations/test_psbmpc_psim_intelligent.cpp)
#add_executable(tester paper_simulations/test_psbmpc_psim.cpp)

target_include_directories(tester PRIVATE
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/libs/
	${EIGEN3_INCLUDE_DIRS}
)

if (USE_GPU_PSBMPC)
	set_property(TARGET tester PROPERTY CUDA_ARCHITECTURES 61 86)
endif()