#======================================================================
# This CMakeLists.txt creates the GPU-version PSB-MPC target. 

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

find_package(CUDA COMPONENTS THRUST REQUIRED)

add_library(psbmpc_lib_gpu STATIC 	
	src/psbmpc.cu 
	src/psbmpc_parameters.cpp
	src/cb_cost_functor.cu
	src/obstacle_manager.cu
	src/obstacle.cu
	#src/obstacle_ship.cu 
	#src/obstacle_sbmpc.cu 
	src/tracked_obstacle.cu
	src/cuda_obstacle.cu
	#src/prediction_obstacle.cu
	src/ownship.cu 
	src/cpe.cu
	src/kf.cu  
	src/mrou.cu 
	src/utilities.cpp
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
	# debug compilation
	#PUBLIC $<$<COMPILE_LANGUAGE:CUDA>: -G -Xcompiler -g -lineinfo -Wall -Wno-pedantic -dc -gencode arch=compute_61,code=sm_61 --expt-relaxed-constexpr -Xlinker -dlink> 

	# release compilation
	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>: -Xcompiler -Wall -Wno-pedantic -dc -gencode arch=compute_61,code=sm_61 --expt-relaxed-constexpr -Xlinker -dlink> 
)

target_link_libraries(psbmpc_lib_gpu 
	PRIVATE ${Matlab_LIBRARIES} 
	PRIVATE ${CUDA_LIBRARIES}
)