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

if(APPLE)
	set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} /opt/homebrew/Cellar")
	message(STATUS "Operating system is Apple OS X")
elseif(UNIX)
	message(STATUS "Operating system is UNIX")
else()
	message(STATUS "Operating system is Windows")
endif()
message(STATUS "CMAKE_MODULE_PATH =${CMAKE_MODULE_PATH}")

# Set default values to PSBMPC flags if they are not set
if(NOT DEFINED USE_GPU_PSBMPC)
	set(USE_GPU_PSBMPC 1)
endif()
message(STATUS "USE_GPU_PSBMPC set to ${USE_GPU_PSBMPC}")

if(USE_GPU_PSBMPC AND NOT DEFINED ${CMAKE_CUDA_ARCHITECTURES})
    set(CMAKE_CUDA_ARCHITECTURES 61 86)
	message(STATUS "CUDA architectures set to ${CMAKE_CUDA_ARCHITECTURES}")
endif()

if(NOT DEFINED OWNSHIP_TYPE)
	set(OWNSHIP_TYPE 0 CACHE BOOL "Sets MPC own-ship model to use with the library.")
endif()
message(STATUS "OWNSHIP_TYPE set to ${OWNSHIP_TYPE}")

if(NOT DEFINED ENABLE_PSBMPC_DEBUGGING)
	set(ENABLE_PSBMPC_DEBUGGING 1)
endif()
message(STATUS "ENABLE_PSBMPC_DEBUGGING set to ${ENABLE_PSBMPC_DEBUGGING}")

if(NOT DEFINED ENABLE_TEST_FILE_PLOTTING)
	set(ENABLE_TEST_FILE_PLOTTING 1)
endif()
message(STATUS "ENABLE_TEST_FILE_PLOTTING set to ${ENABLE_TEST_FILE_PLOTTING}")


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(PSBMPC_LIB_MAJOR_VERSION 1)
set(PSBMPC_LIB_MINOR_VERSION 0)
set(PSBMPC_LIB_VERSION ${PSBMPC_LIB_MAJOR_VERSION}.${PSBMPC_LIB_MINOR_VERSION})

message(STATUS "PSBMPC_LIB_VERSION is ${PSBMPC_LIB_VERSION}")
if(USE_GPU_PSBMPC)
	project(PSBMPC${PSBMPC_LIB_MAJOR_VERSION} VERSION ${PSBMPC_LIB_VERSION} LANGUAGES CUDA CXX)
else()
	project(PSBMPC${PSBMPC_LIB_MAJOR_VERSION} VERSION ${PSBMPC_LIB_VERSION} LANGUAGES CXX)
endif()

add_subdirectory(psbmpc_lib)

add_subdirectory(tests)