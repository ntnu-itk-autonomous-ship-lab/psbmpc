#======================================================================
# This CMakeLists.txt creates the PSB-MPC library target. 
# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

# use the -D compile option to specify USE_GPU_PSBMPC=1 if you want to compile and use
# the GPU version. The default is only compilation of the CPU version
if(USE_GPU_PSBMPC)
	find_package(CUDA COMPONENTS THRUST REQUIRED)

	add_library(psbmpc_lib STATIC 	
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
		src/obstacle_predictor.cpp
		src/obstacle_manager.cpp
		src/tracked_obstacle.cpp
		src/kf.cpp  
		src/mrou.cpp 
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)

	# Use these set commands when building the GPU version 
#	 NOTE: Modify or add the architecture property corresponding to your NVIDIA GPU!
	set_target_properties(psbmpc_lib PROPERTIES 
		CUDA_SEPARABLE_COMPILATION ON
		CUDA_ARCHITECTURES "61;86"
	)

	target_include_directories(psbmpc_lib PUBLIC tryms_matrix ${CUDA_INCLUDE_DIRS})

	target_link_libraries(psbmpc_lib PRIVATE ${CUDA_LIBRARIES})

	message("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
	message("CUDA_LIBARIES: ${CUDA_LIBRARIES}")
else()
	add_library(psbmpc_lib STATIC 	
		src/cpu/psbmpc_cpu.cpp
		src/cpu/cpe_cpu.cpp
		src/cpu/kinetic_ship_models_cpu.cpp
		src/cpu/kinematic_ship_models_cpu.cpp 

		src/psbmpc_parameters.cpp
		src/sbmpc_parameters.cpp 
		src/sbmpc.cpp 
		src/obstacle_predictor.cpp
		src/obstacle_manager.cpp
		src/tracked_obstacle.cpp
		src/kf.cpp  
		src/mrou.cpp 
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)
endif()

find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

target_include_directories(psbmpc_lib 
	PUBLIC
		include
		${Matlab_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
)

target_link_libraries(psbmpc_lib PUBLIC ${Matlab_LIBRARIES} ${Boost_LIBRARIES} Eigen3::Eigen)

message("Use GPU PSBMPC: ${USE_GPU_PSBMPC}")
message("Own-ship type: ${OWNSHIP_TYPE}")
message("Enable PSBMPC debugging: ${ENABLE_PSBMPC_DEBUGGING}")

target_compile_options(psbmpc_lib
	PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -std=c++17 -Wall -Wextra -Wno-deprecated-copy -DUSE_GPU_PSBMPC=${USE_GPU_PSBMPC} -DOWNSHIP_TYPE=${OWNSHIP_TYPE} -DENABLE_PSBMPC_DEBUGGING=${ENABLE_PSBMPC_DEBUGGING} -DEIGEN_MAX_STATIC_ALIGN_BYTES=16>
	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  --disable-warnings -Xcompiler -dc --expt-relaxed-constexpr -DUSE_GPU_PSBMPC=${USE_GPU_PSBMPC} -DOWNSHIP_TYPE=${OWNSHIP_TYPE} -DENABLE_PSBMPC_DEBUGGING=${ENABLE_PSBMPC_DEBUGGING} -DEIGEN_MAX_STATIC_ALIGN_BYTES=16  -Xlinker -dlink>
)

#if (USE_GPU_PSBMPC)
#	target_compile_options(psbmpc_lib
#		PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -DUSE_GPU_PSBMPC=1>
#		PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -DUSE_GPU_PSBMPC=1>
#	)
#endif()

#if (ENABLE_PSBMPC_DEBUGGING)
#	target_compile_options(psbmpc_lib
#		PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -DENABLE_PSBMPC_DEBUGGING=1>
#		PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -DENABLE_PSBMPC_DEBUGGING=1>
#	)
#endif()

if (OWNSHIP_TYPE EQUAL 0)
	target_compile_options(psbmpc_lib
		PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -DOWNSHIP_TYPE=0>
		PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -DOWNSHIP_TYPE=0>
	)
elseif(OWNSHIP_TYPE EQUAL 1)
	target_compile_options(psbmpc_lib
		PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -DOWNSHIP_TYPE=1>
		PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -DOWNSHIP_TYPE=1>
	)
elseif(OWNSHIP_TYPE EQUAL 2)
	target_compile_options(psbmpc_lib
		PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -DOWNSHIP_TYPE=2>
		PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -DOWNSHIP_TYPE=2>
	)
endif()

if(CMAKE_BUILD_TYPE EQUAL release)
	target_compile_options(psbmpc_lib PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -j4>)
else()
	# Host and device debug compilation and profiling, comment out if you want to compile with debugging symbols etc.
	#set(CMAKE_VERBOSE_MAKEFILE ON)
	#target_compile_options(psbmpc_lib
	#	PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>
	#	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  -Xcompiler -g -G -lineinfo -Wall -Wno-pedantic>)
endif()


## Create package for the library (NOT FINALIZED, NEED CORRECTIONS)
#file(
#    WRITE "${CMAKE_CURRENT_BINARY_DIR}/Config.cmake.in"
#    "
#        @PACKAGE_INIT@
#		include( \"\$\{CMAKE_CURRENT_LIST_DIR\}/psbmpc_libTargets.cmake\" )
#
#		include(CMakeFindDependencyMacro)
#		find_dependency(Eigen3)
#		find_dependency(Matlab)
#    "
#)

#include(GNUInstallDirs)
#install(TARGETS psbmpc_lib
#	EXPORT psbmpc_libTargets
#	LIBRARY DESTINATION lib
#	ARCHIVE DESTINATION lib
#	RUNTIME DESTINATION bin
#	INCLUDES DESTINATION include 
#)

#install(
#	EXPORT psbmpc_libTargets
#	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/psbmpc_lib
#	NAMESPACE psbmpc_lib::
#	FILE psbmpc_libTargets.cmake
#)

#include(CMakePackageConfigHelpers)
#configure_package_config_file( 
#  "${CMAKE_CURRENT_BINARY_DIR}/Config.cmake.in" 
#  "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_libConfig.cmake"
#  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/psbmpc_lib
#  PATH_VARS CMAKE_INSTALL_LIBDIR
#)

#write_basic_package_version_file(
#  "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_libConfigVersion.cmake"
#  VERSION 1.0.0
#  COMPATIBILITY AnyNewerVersion
#)

#install(
#  FILES
#    "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_libConfig.cmake"
#    "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_libConfigVersion.cmake"
#  DESTINATION
#    ${CMAKE_INSTALL_LIBDIR}/cmake/psbmpc_lib
#  COMPONENT
#    Devel
#)