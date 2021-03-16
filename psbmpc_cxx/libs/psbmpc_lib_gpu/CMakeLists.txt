#======================================================================
# This CMakeLists.txt creates the PSB-MPC library target. 

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

find_package(CUDA COMPONENTS THRUST REQUIRED)

add_library(psbmpc_lib_gpu STATIC 	
	src/cpu/psbmpc_cpu.cpp
	src/cpu/cpe_cpu.cpp
	src/cpu/obstacle_ship_cpu.cpp 
	src/cpu/obstacle_sbmpc_cpu.cpp
	src/cpu/ownship_cpu.cpp
	src/cpu/prediction_obstacle_cpu.cpp

	src/gpu/psbmpc_gpu.cu 
	src/gpu/cpe_gpu.cu 
	src/gpu/obstacle_ship_gpu.cu 
	src/gpu/obstacle_sbmpc_gpu.cu
	src/gpu/obstacle_sbmpc_parameters_gpu.cu
	src/gpu/ownship_gpu.cu
	src/gpu/prediction_obstacle_gpu.cu
	src/gpu/cuda_obstacle.cu
	src/gpu/cb_cost_functor.cu 
	
	src/psbmpc_parameters.cpp
	src/sbmpc_parameters.cpp 
	src/sbmpc.cpp 
	src/joint_prediction_manager.cpp
	src/obstacle_manager.cpp
	src/tracked_obstacle.cpp
	src/kf.cpp  
	src/mrou.cpp 
)

set_target_properties(psbmpc_lib_gpu PROPERTIES
	CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(psbmpc_lib_gpu PUBLIC 
	include
	tryms_matrix
	${CMAKE_SOURCE_DIR}/libs/third_party_libs
	${Matlab_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
)

message("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
message("CUDA_LIBARIES: ${CUDA_LIBRARIES}")

target_compile_options(psbmpc_lib_gpu 
	PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: -DEIGEN_MAX_STATIC_ALIGN_BYTES=16>
	# debug compilation
	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>: -G -Xcompiler -g -lineinfo -Wall -Wno-pedantic -dc --expt-relaxed-constexpr -Xlinker -dlink> 

	# release compilation
	#PUBLIC $<$<COMPILE_LANGUAGE:CUDA>: -Xcompiler -Wall -Wno-pedantic -dc --expt-relaxed-constexpr -Xlinker -dlink> 
)

target_link_libraries(psbmpc_lib_gpu 
	PRIVATE ${Matlab_LIBRARIES} 
	PRIVATE ${CUDA_LIBRARIES}
)