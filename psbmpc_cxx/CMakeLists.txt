#======================================================================
# This CMakeLists.txt sets up CMake build-file generation for the
# usage of the psb-mpc library. 
# Eigen and Matlab is required for both the CPU- and GPU version, and
# CUDA and Thrust is required for the GPU-version.

# This file connects the PSBMPC library target, and the test file
# target (both objects in CMake sense).







# Uncomment the add_subdirectory and target_link_libraries commands 
# that correspond to the library version you want to use.

# ---------------------------------------------------------------------

# Version 1.0

# Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
# All rights reserved.

# Author 	: Trym Tengesdal

# Modified 	: 
#======================================================================

cmake_minimum_required(VERSION 3.17.3)
project(psbmpc_test LANGUAGES CUDA CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

# for debug compilation and profiling
add_compile_options(-Wall -Wextra -fno-omit-frame-pointer -g -DEIGEN_MAX_STATIC_ALIGN_BYTES=16)

#add_compile_options(-Wall -Wextra -Wno-deprecated-copy -DEIGEN_MAX_STATIC_ALIGN_BYTES=16)

add_subdirectory(libs/psbmpc_lib_gpu)
#add_subdirectory(libs/psbmpc_lib)

add_subdirectory(src/tests)

# Use these set commands when building the GPU version
set_property(TARGET psbmpc_lib_gpu PROPERTY CUDA_ARCHITECTURES 61 86)
set_property(TARGET tester PROPERTY CUDA_ARCHITECTURES 61 86)

target_link_libraries(tester PRIVATE psbmpc_lib_gpu)
#target_link_libraries(tester PRIVATE psbmpc_lib)


#message("Your C++ compiler supports these C++ features:")
#foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
#  message("${i}")
#endforeach()