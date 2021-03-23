#======================================================================
# This CMakeLists.txt creates the PSB-MPC library target. 
# If you want 
# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

# use the -D flag to specify USE_GPU_PSBMPC=1 if you want to compile and use
# the GPU version. The default is only compilation of the CPU version
if(USE_GPU_PSBMPC)
	find_package(CUDA COMPONENTS THRUST REQUIRED)

	add_library(psbmpc_lib STATIC 	
		src/cpu/psbmpc_cpu.cpp
		src/cpu/cpe_cpu.cpp
		src/cpu/kinetic_ship_models_cpu.cpp
		src/cpu/kinematic_ship_models_cpu.cpp 
		src/cpu/obstacle_sbmpc_cpu.cpp
		src/cpu/prediction_obstacle_cpu.cpp

		src/gpu/psbmpc_gpu.cu 
		src/gpu/cpe_gpu.cu 
		src/gpu/kinetic_ship_models_gpu.cu
		src/gpu/kinematic_ship_models_gpu.cu
		src/gpu/obstacle_sbmpc_gpu.cu
		src/gpu/obstacle_sbmpc_parameters_gpu.cu
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
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)

	set_target_properties(psbmpc_lib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

	target_include_directories(psbmpc_lib PUBLIC tryms_matrix PRIVATE ${CUDA_INCLUDE_DIRS})

	target_link_libraries(psbmpc_lib PRIVATE ${CUDA_LIBRARIES})

	message("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
	message("CUDA_LIBARIES: ${CUDA_LIBRARIES}")
else()
	add_library(psbmpc_lib STATIC 	
		src/cpu/psbmpc_cpu.cpp
		src/cpu/cpe_cpu.cpp
		src/cpu/kinetic_ship_models_cpu.cpp
		src/cpu/kinematic_ship_models_cpu.cpp 
		src/cpu/obstacle_sbmpc_cpu.cpp
		src/cpu/prediction_obstacle_cpu.cpp

		src/psbmpc_parameters.cpp
		src/sbmpc_parameters.cpp 
		src/sbmpc.cpp 
		src/joint_prediction_manager.cpp
		src/obstacle_manager.cpp
		src/tracked_obstacle.cpp
		src/kf.cpp  
		src/mrou.cpp 
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)
endif()


find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

target_include_directories(psbmpc_lib PUBLIC 
	include
	${CMAKE_SOURCE_DIR}/libs/third_party_libs
	${Matlab_INCLUDE_DIRS}
)

target_link_libraries(psbmpc_lib PRIVATE ${Matlab_LIBRARIES})

target_compile_options(psbmpc_lib 
	PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -DEIGEN_MAX_STATIC_ALIGN_BYTES=16>
	PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: -DEIGEN_MAX_STATIC_ALIGN_BYTES=16>

	# debug compilation
	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>: -G -Xcompiler -g -lineinfo -Wall -Wno-pedantic -dc --expt-relaxed-constexpr -Xlinker -dlink> 

	# release compilation
	#PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: -Xcompiler -Wall -Wno-pedantic -dc --expt-relaxed-constexpr -Xlinker -dlink> 
)

# Use these set commands when building the GPU version
set_property(TARGET psbmpc_lib PROPERTY CUDA_ARCHITECTURES 61 86)
#set_property(TARGET tester PROPERTY CUDA_ARCHITECTURES 61 86)