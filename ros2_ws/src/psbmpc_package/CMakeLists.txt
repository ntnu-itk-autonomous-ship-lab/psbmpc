cmake_minimum_required(VERSION 3.5)
project(psbmpc_package LANGUAGES CUDA CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # debug:
  #add_compile_options(-fno-omit-frame-pointer -g)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(psbmpc_interfaces REQUIRED)
find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies 
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  nav_msgs
  psbmpc_interfaces
  Matlab
  Eigen3
)

# Toggle usage of the CPU(0)/GPU(1) version of PSBMPC
set(USE_GPU_PSBMPC 1)

# OWNSHIP_TYPE = 0 : Kinematic_Ship | = 1 : Telemetron | = 2 : MilliAmpere(NOT FINISHED YET)
set(OWNSHIP_TYPE 0)

# Toggle usage of PSBMPC debugging version of PSBMPC
set(ENABLE_PSBMPC_DEBUGGING 0)

add_subdirectory(libs/psbmpc_lib)

add_executable(psbmpc_node src/psbmpc_node.cpp)

ament_target_dependencies(psbmpc_node ${dependencies})

target_link_libraries(psbmpc_node psbmpc_lib ${Matlab_LIBRARIES} Eigen3::Eigen)

target_include_directories(psbmpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(psbmpc_node PROPERTIES 
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_ARCHITECTURES "61;86"
)

#rclcpp_components_register_nodes(psbmpc_node "PSB-MPC")

#ament_export_targets(export_psbmpc_node HAS_LIBRARY_TARGET)
#ament_export_dependencies(${dependencies})

install(TARGETS 
  psbmpc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()