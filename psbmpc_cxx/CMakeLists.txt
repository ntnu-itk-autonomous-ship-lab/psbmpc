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

# Set default values to PSBMPC flags if they are not set
if(NOT DEFINED USE_GPU_PSBMPC)
	set(USE_GPU_PSBMPC 1)
endif()
message(STATUS "USE_GPU_PSBMPC set to ${USE_GPU_PSBMPC}")

if(NOT DEFINED ${CMAKE_CUDA_ARCHITECTURES})
    set(CMAKE_CUDA_ARCHITECTURES 61 86)
endif()
message(STATUS "CUDA architectures set to ${CMAKE_CUDA_ARCHITECTURES}")

if(NOT DEFINED OWNSHIP_TYPE)
	set(OWNSHIP_TYPE 0)
endif()
message(STATUS "OWNSHIP_TYPE set to ${OWNSHIP_TYPE}")

if(NOT DEFINED ENABLE_PSBMPC_DEBUGGING)
	set(ENABLE_PSBMPC_DEBUGGING 0)
endif()
message(STATUS "ENABLE_PSBMPC_DEBUGGING set to ${ENABLE_PSBMPC_DEBUGGING}")

set(PSBMPC_LibVersion 1.0)
if(USE_GPU_PSBMPC)
	project(PSBMPC_Library VERSION ${PSBMPC_LibVersion} LANGUAGES CUDA CXX)
else()
	project(PSBMPC_Library VERSION ${PSBMPC_LibVersion} LANGUAGES CXX)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

add_subdirectory(psbmpc_lib)
add_subdirectory(tests)

include(GenerateExportHeader)
generate_export_header(PSBMPC_Lib)

include(GNUInstallDirs)
install(TARGETS PSBMPC_Lib
	EXPORT PSBMPC_LibTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/psbmpc_lib-${PSBMPC_LibVersion}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/psbmpc_lib-${PSBMPC_LibVersion}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/psbmpc_lib-${PSBMPC_LibVersion}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/psbmpc_lib-${PSBMPC_LibVersion}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_lib-${PSBMPC_LibVersion}/PSBMPC_LibConfigVersion.cmake"
  VERSION ${PSBMPC_LibVersion}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT PSBMPC_LibTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_lib-${PSBMPC_LibVersion}/PSBMPC_LibTargets.cmake"
  NAMESPACE PSBMPC_Lib1::
)

configure_file(cmake/PSBMPC_LibConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_lib-${PSBMPC_LibVersion}/PSBMPC_LibConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/psbmpc_lib-${PSBMPC_LibVersion})
install(EXPORT PSBMPC_LibTargets
  FILE
  	PSBMPC_LibTargets.cmake
  NAMESPACE
  	PSBMPC_Lib1::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/PSBMPC_LibConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/psbmpc_lib-${PSBMPC_LibVersion}/PSBMPC_LibConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)