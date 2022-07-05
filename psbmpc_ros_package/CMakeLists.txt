cmake_minimum_required(VERSION 3.13.1)

# Toggle usage of the CPU(0)/GPU(1) version of PSBMPC
# set(USE_GPU_PSBMPC 1)

# OWNSHIP_TYPE = 0 : Kinematic_Ship | = 1 : Telemetron
set(OWNSHIP_TYPE 0)

# Toggle usage of debugging in the PSBMPC
set(ENABLE_PSBMPC_DEBUGGING 0)

if(USE_GPU_PSBMPC)
  project(psbmpc LANGUAGES CUDA CXX)
else()
  project(psbmpc LANGUAGES CXX)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  message_filters
  geometry_msgs
  nav_msgs
  custom_msgs
  ros_af_msgs
)

find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PSBMPC1 REQUIRED)

add_service_files(
  DIRECTORY
  srv
)

generate_messages(
  DEPENDENCIES
  custom_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib geometry_msgs nav_msgs custom_msgs ros_af_msgs message_runtime message_filters
)

add_executable(${PROJECT_NAME}_node src/psbmpc_node.cpp)
add_executable(dom_node src/dom_node.cpp)
add_executable(ghm_node src/ghm_node.cpp)
add_executable(psbmpc_visualization_node src/psbmpc_visualization_node.cpp)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  custom_msgs_generate_messages_cpp
  ros_af_msgs_generate_messages_cpp
)
add_dependencies(dom_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  custom_msgs_generate_messages_cpp
)
add_dependencies(ghm_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  custom_msgs_generate_messages_cpp
)
add_dependencies(psbmpc_visualization_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  custom_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE PSBMPC1::PSBMPC Eigen3::Eigen GeographicLib yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(dom_node PRIVATE PSBMPC1::PSBMPC Eigen3::Eigen GeographicLib ${catkin_LIBRARIES})
target_link_libraries(ghm_node PRIVATE PSBMPC1::PSBMPC Eigen3::Eigen GeographicLib ${catkin_LIBRARIES})
target_link_libraries(psbmpc_visualization_node PRIVATE PSBMPC1::PSBMPC Eigen3::Eigen GeographicLib ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${custom_msgs_INCLUDE_DIRS}
)
target_include_directories(dom_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${custom_msgs_INCLUDE_DIRS}
)
target_include_directories(ghm_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${custom_msgs_INCLUDE_DIRS}
)
target_include_directories(psbmpc_visualization_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${custom_msgs_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Wall -Wextra -Wpedantic>)
target_compile_options(dom_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Wall -Wextra -Wpedantic>)
target_compile_options(ghm_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Wall -Wextra -Wpedantic>)
target_compile_options(psbmpc_visualization_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Wall -Wextra -Wpedantic>)

message(STATUS "CMAKE_BUILD_TYPE PSBMPC NODE =${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "debug")
  target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  target_compile_options(dom_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  target_compile_options(ghm_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  target_compile_options(psbmpc_visualization_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
elseif(CMAKE_BUILD_TYPE STREQUAL "release")
  target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  target_compile_options(dom_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  target_compile_options(ghm_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  target_compile_options(psbmpc_visualization_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -fno-omit-frame-pointer -g>)
  # target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -O3>)
  # target_compile_options(dom_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -O3>)
  # target_compile_options(ghm_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -O3>)
  # target_compile_options(psbmpc_visualization_node PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -O3>)
endif()

if(USE_GPU_PSBMPC)
  set_target_properties(${PROJECT_NAME}_node PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES "61;86"
  )
endif()