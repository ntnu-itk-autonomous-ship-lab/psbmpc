cmake_minimum_required(VERSION 3.5)
project(bridge_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Trajectory2.msg"
  "msg/Trajectory4.msg"
  "msg/Reference.msg"
  "msg/Offset.msg"
  "msg/Vector2.msg"
  "msg/Vector4.msg"
  "msg/Covariance2.msg"
  "msg/CrossCovariance2.msg"
  "msg/KinematicEstimate.msg"
  "msg/KinematicEstimateStamped.msg"
  "msg/DynamicObstacleEstimates.msg"
  "msg/DynamicObstacleEstimatesStamped.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
 )

 ament_export_dependencies(rosidl_default_runtime)

install(
  FILES rosbridge_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
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