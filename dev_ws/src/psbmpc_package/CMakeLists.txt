cmake_minimum_required(VERSION 3.5)
project(psbmpc_package LANGUAGES CUDA CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # debug:
  #add_compile_options(-fno-omit-frame-pointer -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psbmpc_interfaces REQUIRED)

add_subdirectory(src/libs/psbmpc_lib)

add_executable(psbmpc_node src/psbmpc_node.cpp)

ament_target_dependencies(psbmpc_node 
  rclcpp 
  psbmpc_interfaces)

target_link_libraries(psbmpc_node PRIVATE psbmpc_lib)

target_include_directories(psbmpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS psbmpc_node
  DESTINATION lib/${PROJECT_NAME})

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