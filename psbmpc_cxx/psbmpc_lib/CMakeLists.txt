#======================================================================
# This CMakeLists.txt creates and installs the PSB-MPC library target. 
# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================
cmake_minimum_required(VERSION 3.17.3)

set(LIBRARY_TARGET_NAME PSBMPC_Lib)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(GeographicLib REQUIRED) # Specify CMAKE_PREFIX_PATH=/path/to/geographiclib/builddir for cmake to find the package config files
if(ENABLE_PSBMPC_DEBUGGING) # Neccessary for running most of the psbmpc test files
	find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)
endif()

# use the -D compile option to specify USE_GPU_PSBMPC=1 if you want to compile and use
# the GPU version. The default is only compilation of the CPU version
if(USE_GPU_PSBMPC)
	find_package(CUDA COMPONENTS THRUST REQUIRED)

	set(${LIBRARY_TARGET_NAME}_HDR
		include/cpu/cpe_cpu.hpp
		include/cpu/kinematic_ship_models_cpu.hpp
		include/cpu/kinetic_ship_models_cpu.hpp
		include/cpu/mpc_cost_cpu.hpp
		include/cpu/psbmpc_cpu.hpp
		include/cpu/utilities_cpu.hpp
		
		include/gpu/cb_cost_functor_structures.cuh
		include/gpu/cb_cost_functor.cuh
		include/gpu/cpe_gpu.cuh
		include/gpu/cuda_obstacle.cuh
		include/gpu/kinematic_ship_models_gpu.cuh
		include/gpu/kinetic_ship_models_gpu.cuh
		include/gpu/mpc_cost_gpu.cuh
		include/gpu/psbmpc_gpu.cuh
		include/gpu/utilities_gpu.cuh

		include/gpu/tml/tml.cuh
		include/gpu/tml/dynamic_matrix.cuh
		include/gpu/tml/static_matrix.cuh
		include/gpu/tml/pseudo_dynamic_matrix.cuh
		include/gpu/tml/eigen_interface.cuh

		include/grounding_hazard_manager.hpp
		include/kf.hpp
		include/mrou.hpp
		include/obstacle_manager.hpp
		include/obstacle_predictor.hpp
		include/psbmpc_defines.hpp
		include/psbmpc_index.hpp
		include/psbmpc_parameters.hpp
		include/sbmpc_index.hpp
		include/sbmpc_parameters.hpp
		include/sbmpc.hpp
		include/tracked_obstacle.hpp
		include/utm_projection.hpp
		include/xoshiro.hpp
		include/shapefile_related/shapefil.h
	)

	set(${LIBRARY_TARGET_NAME}_SRC
		src/cpu/psbmpc_cpu.cpp
		src/cpu/cpe_cpu.cpp
		src/cpu/kinetic_ship_models_cpu.cpp
		src/cpu/kinematic_ship_models_cpu.cpp 

		src/gpu/psbmpc_gpu.cu 
		src/gpu/cpe_gpu.cu 
		src/gpu/kinetic_ship_models_gpu.cu
		src/gpu/kinematic_ship_models_gpu.cu
		src/gpu/cuda_obstacle.cu
		src/gpu/cb_cost_functor.cu 
		
		src/psbmpc_parameters.cpp
		src/sbmpc_parameters.cpp 
		src/sbmpc.cpp 
		src/obstacle_manager.cpp
		src/tracked_obstacle.cpp
		src/kf.cpp  
		src/mrou.cpp 
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)
else()
	set(${LIBRARY_TARGET_NAME}_HDR
		include/cpu/cpe_cpu.hpp
		include/cpu/kinematic_ship_models_cpu.hpp
		include/cpu/kinetic_ship_models_cpu.hpp
		include/cpu/mpc_cost_cpu.hpp
		include/cpu/psbmpc_cpu.hpp
		include/cpu/utilities_cpu.hpp

		include/grounding_hazard_manager.hpp
		include/kf.hpp
		include/mrou.hpp
		include/obstacle_manager.hpp
		include/obstacle_predictor.hpp
		include/psbmpc_defines.hpp
		include/psbmpc_index.hpp
		include/psbmpc_parameters.hpp
		include/sbmpc_index.hpp
		include/sbmpc_parameters.hpp
		include/sbmpc.hpp
		include/tracked_obstacle.hpp
		include/utm_projection.hpp
		include/xoshiro.hpp
		include/shapefile_related/shapefil.h
	)

	set(${LIBRARY_TARGET_NAME}_SRC
		src/cpu/psbmpc_cpu.cpp
		src/cpu/cpe_cpu.cpp
		src/cpu/kinetic_ship_models_cpu.cpp
		src/cpu/kinematic_ship_models_cpu.cpp 

		src/psbmpc_parameters.cpp
		src/sbmpc_parameters.cpp 
		src/sbmpc.cpp 
		src/obstacle_manager.cpp
		src/tracked_obstacle.cpp
		src/kf.cpp  
		src/mrou.cpp 
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)
endif()

add_library(${LIBRARY_TARGET_NAME} STATIC ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES PUBLIC_HEADER ${${LIBRARY_TARGET_NAME}_HDR})

if(USE_GPU_PSBMPC)
	set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
	target_link_libraries(${LIBRARY_TARGET_NAME} INTERFACE CUDA::THRUST)
endif()

if(ENABLE_PSBMPC_DEBUGGING)
	target_link_libraries(${LIBRARY_TARGET_NAME} INTERFACE Matlab::MX_LIBRARY Matlab::ENG_LIBRARY)
endif(ENABLE_PSBMPC_DEBUGGING)

target_include_directories(${LIBRARY_TARGET_NAME} 
	PRIVATE
		include
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${LIBRARY_TARGET_NAME} INTERFACE Eigen3::Eigen Boost GeographicLib)

message("Use GPU PSBMPC: ${USE_GPU_PSBMPC}")
message("Own-ship type: ${OWNSHIP_TYPE}")
message("Enable PSBMPC debugging (Using MATLAB): ${ENABLE_PSBMPC_DEBUGGING}")

target_compile_options(${LIBRARY_TARGET_NAME}
	PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -std=c++17 -Wall -Wextra -Wno-deprecated-copy -DUSE_GPU_PSBMPC=${USE_GPU_PSBMPC} -DOWNSHIP_TYPE=${OWNSHIP_TYPE} -DENABLE_PSBMPC_DEBUGGING=${ENABLE_PSBMPC_DEBUGGING} -DEIGEN_MAX_STATIC_ALIGN_BYTES=16>
	PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:  --disable-warnings -Xcompiler -dc --expt-relaxed-constexpr -DUSE_GPU_PSBMPC=${USE_GPU_PSBMPC} -DOWNSHIP_TYPE=${OWNSHIP_TYPE} -DENABLE_PSBMPC_DEBUGGING=${ENABLE_PSBMPC_DEBUGGING} -DEIGEN_MAX_STATIC_ALIGN_BYTES=16  -Xlinker -dlink>
)

if(CMAKE_BUILD_TYPE EQUAL release)
	target_compile_options(${LIBRARY_TARGET_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -j4>)
else()
	# Host and device debug compilation and profiling, comment out if you want to compile with debugging symbols etc.
	#set(CMAKE_VERBOSE_MAKEFILE ON)
	#target_compile_options(PSBMPC_Lib
	#	PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>
	#	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -Xcompiler -g -G -lineinfo -Wall -Wno-pedantic>)
endif()