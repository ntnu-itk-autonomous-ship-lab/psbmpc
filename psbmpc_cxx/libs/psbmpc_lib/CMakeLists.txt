#======================================================================
# This CMakeLists.txt creates the CPU-version PSB-MPC target. 

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)
find_package(Boost)

add_library(psbmpc_lib STATIC 
	src/psbmpc.cpp 
	src/psbmpc_parameters.cpp
	src/sbmpc.cpp 
	src/sbmpc_parameters.cpp
	src/ownship.cpp 
	src/cpe.cpp 
	src/kf.cpp 
	src/obstacle_manager.cpp
	src/obstacle.cpp 
	src/obstacle_ship.cpp 
	src/obstacle_sbmpc.cpp 
	src/joint_prediction_manager.cpp
	src/tracked_obstacle.cpp
	src/prediction_obstacle.cpp 
	src/mrou.cpp 
	src/utilities.cpp
	src/shapefile_related/safileio.cpp
	src/shapefile_related/shpopen.cpp)

target_include_directories(psbmpc_lib PUBLIC 
	include
	${CMAKE_SOURCE_DIR}/libs/third_party_libs
	${Matlab_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	)

target_compile_features(psbmpc_lib PRIVATE cxx_std_11)

target_link_libraries(psbmpc_lib ${Matlab_LIBRARIES})