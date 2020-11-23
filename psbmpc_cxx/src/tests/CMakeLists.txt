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

# Test files for use with CPU-version of PSBMPC
#add_executable(tester test_psbmpc.cpp)
#add_executable(tester test_ownship.cpp)
#add_executable(tester test_obstacle_ship.cpp)
#add_executable(tester test_obstacle.cpp)
#add_executable(tester test_cpe.cpp)
#add_executable(tester test_mrou.cpp)
#add_executable(tester test_kf.cpp)
#add_executable(tester test_prng.cpp)

# Test files for use with GPU-version of PSBMPC
add_executable(tester gpu_implementation_test_files/test_psbmpc.cu)
#add_executable(tester gpu_implementation_test_files/test_ownship.cu)
#add_executable(tester gpu_implementation_test_files/test_obstacle_ship.cu)
#add_executable(tester gpu_implementation_test_files/test_cpe.cu)
#add_executable(tester gpu_implementation_test_files/stack_overflow.cu)
#add_executable(tester gpu_implementation_test_files/test_thrust.cu)
#add_executable(tester gpu_implementation_test_files/test_tml.cu)
#add_executable(tester gpu_implementation_test_files/test_tml_pdmatrix.cu)

#add_executable(tester paper_simulations/test_psbmpc_psim.cpp)

target_include_directories(tester PRIVATE
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/libs/
)