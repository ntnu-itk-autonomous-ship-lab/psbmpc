#======================================================================
# This CMakeLists.txt creates the target for use in testing PSB-MPC
# functionality. Almost every module of the PSB-MPC lib can be tested, 
# by uncommenting the module test file you want to select. 

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

find_package(Eigen3 REQUIRED)
find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

# Pure host side test files
#add_executable(tester test_psbmpc.cpp)
add_executable(tester test_psbmpc_grounding.cpp)
#add_executable(tester test_ownship_cpu.cpp)
#add_executable(tester test_obstacle_ship_cpu.cpp)
#add_executable(tester test_obstacle_cpu.cpp)
#add_executable(tester test_obstacle_predictor.cpp)
#add_executable(tester test_cpe_cpu.cpp)
#add_executable(tester test_mrou.cpp)
#add_executable(tester test_kf.cpp)
#add_executable(tester test_prng.cpp)
#add_executable(tester test_psbmpc_cost_class.cpp)
#add_executable(tester test_read_shapefile.cpp)
#add_executable(tester test_polygons.cpp)

# Test files specific to the GPU-version of the PSBMPC
#add_executable(tester test_ownship_gpu.cu)
#add_executable(tester test_obstacle_ship_gpu.cu)
#add_executable(tester test_cpe_gpu.cu)
#add_executable(tester test_thrust.cu)
#add_executable(tester test_tml.cu)
#add_executable(tester test_tml_pdmatrix.cu)

target_include_directories(tester PRIVATE
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/libs/
	${EIGEN3_INCLUDE_DIRS}
	${Matlab_INCLUDE_DIRS}
)

target_link_libraries(tester PRIVATE ${Matlab_LIBRARIES})

if (USE_GPU_PSBMPC)
	set_target_properties(tester PROPERTIES 
		CUDA_SEPARABLE_COMPILATION ON
		CUDA_ARCHITECTURES "61;86")
endif()
