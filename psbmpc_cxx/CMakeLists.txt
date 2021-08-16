#======================================================================
# This CMakeLists.txt sets up CMake build-file generation for the
# usage of the psb-mpc library. 

# It connects the PSBMPC library target, and the test file
# target (both objects in CMake sense).

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

cmake_minimum_required(VERSION 3.17.3)

if(USE_GPU_PSBMPC)
	project(PSBMPC LANGUAGES CUDA CXX)
else()
	project(PSBMPC LANGUAGES CXX)
endif()

# for host side debug compilation and profiling
#set(CMAKE_VERBOSE_MAKEFILE ON)
#add_compile_options(-fno-omit-frame-pointer -g)

add_subdirectory(libs/psbmpc_lib)

add_subdirectory(src/tests)

target_link_libraries(tester PRIVATE psbmpc_lib)