cmake_minimum_required(VERSION 3.17.3)

include(GNUInstallDirs)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

set(LIBRARY_TARGET_NAME PSBMPC)
set(PSBMPC_PACKAGE_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/psbmpc${PSBMPC_LIB_MAJOR_VERSION}/cmake)
set(PSBMPC_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/psbmpc${PSBMPC_LIB_MAJOR_VERSION})

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(GeographicLib REQUIRED)
if(ENABLE_PSBMPC_DEBUGGING OR ENABLE_TEST_FILE_PLOTTING)
	message(STATUS "MATLAB ENABLED...")
	find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)
endif()
message(STATUS "BOOST INCLUDE DIRS: ${Boost_INCLUDE_DIR}")
message(STATUS "BOOST LIBRARIES: ${Boost_LIBRARIES}")

if(USE_GPU_PSBMPC)
	find_package(CUDA COMPONENTS THRUST REQUIRED)

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
		src/tracked_obstacle.cpp
		src/kf.cpp
		src/mrou.cpp
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)
else()
	set(${LIBRARY_TARGET_NAME}_SRC
		src/cpu/psbmpc_cpu.cpp
		src/cpu/cpe_cpu.cpp
		src/cpu/kinetic_ship_models_cpu.cpp
		src/cpu/kinematic_ship_models_cpu.cpp

		src/psbmpc_parameters.cpp
		src/sbmpc_parameters.cpp
		src/sbmpc.cpp
		src/tracked_obstacle.cpp
		src/kf.cpp
		src/mrou.cpp
		src/shapefile_related/safileio.cpp
		src/shapefile_related/shpopen.cpp
	)
endif()

add_library(${LIBRARY_TARGET_NAME} STATIC ${${LIBRARY_TARGET_NAME}_SRC})

if(USE_GPU_PSBMPC)
	set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
	target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
	target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${CUDA_LIBRARIES})
endif()

if(ENABLE_PSBMPC_DEBUGGING OR ENABLE_TEST_FILE_PLOTTING)
	message(STATUS "LINKING MATLAB LIBRARIES...")
	target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC ${Matlab_INCLUDE_DIRS})
	target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${Matlab_LIBRARIES})
endif()

target_include_directories(${LIBRARY_TARGET_NAME}
	PRIVATE include
	PUBLIC
		${Boost_INCLUDE_DIR}
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/psbmpc${PSBMPC_LIB_MAJOR_VERSION}>)

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC Eigen3::Eigen ${Boost_LIBRARIES} GeographicLib)

target_compile_options(${LIBRARY_TARGET_NAME}
	PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -Wall -Wextra -Wno-deprecated-copy -DUSE_GPU_PSBMPC=${USE_GPU_PSBMPC} -DOWNSHIP_TYPE=${OWNSHIP_TYPE} -DENABLE_PSBMPC_DEBUGGING=${ENABLE_PSBMPC_DEBUGGING} -DENABLE_TEST_FILE_PLOTTING=${ENABLE_TEST_FILE_PLOTTING} -DEIGEN_MAX_STATIC_ALIGN_BYTES=16>
	PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:  --disable-warnings -Xcompiler -dc --expt-relaxed-constexpr -DUSE_GPU_PSBMPC=${USE_GPU_PSBMPC} -DOWNSHIP_TYPE=${OWNSHIP_TYPE} -DENABLE_PSBMPC_DEBUGGING=${ENABLE_PSBMPC_DEBUGGING} -DENABLE_TEST_FILE_PLOTTING=${ENABLE_TEST_FILE_PLOTTING} -DEIGEN_MAX_STATIC_ALIGN_BYTES=16  -Xlinker -dlink>
)

message(STATUS "CMAKE BUILD TYPE: ${CMAKE_BUILD_TYPE}")
if(CMAKE_BUILD_TYPE STREQUAL "release")
	target_compile_options(${LIBRARY_TARGET_NAME}
		PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -O3>
		PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: -Xptxas -O3>)
elseif(CMAKE_BUILD_TYPE STREQUAL "debug")
	target_compile_options(${LIBRARY_TARGET_NAME}
		PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>
		#PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:  -Xcompiler -g -G -Wall -Wno-pedantic>
	)
endif()

# INSTALL LIBRARY
include(CMakePackageConfigHelpers)
install(DIRECTORY include/ DESTINATION ${PSBMPC_INCLUDE_DIR} COMPONENT Devel)

install(
	TARGETS ${LIBRARY_TARGET_NAME}
	EXPORT ${PROJECT_NAME}Targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/psbmpc${PSBMPC_LIB_MAJOR_VERSION}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/psbmpc${PSBMPC_LIB_MAJOR_VERSION}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/psbmpc${PSBMPC_LIB_MAJOR_VERSION}
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${PSBMPC_PACKAGE_INSTALL_DIR}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PSBMPC_LIB_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${LIBRARY_TARGET_NAME} NAMESPACE ${PROJECT_NAME}:: FILE ${PROJECT_NAME}Targets.cmake)
export(PACKAGE ${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}:: DESTINATION ${PSBMPC_PACKAGE_INSTALL_DIR} COMPONENT Devel)
install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	DESTINATION
		${PSBMPC_PACKAGE_INSTALL_DIR}
	COMPONENT
		Devel
)