cmake_minimum_required(version 3.10)
project(psbmpc_test)


## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)


add_executable(psbmpc_tester src/test.cpp src/psbmpc_lib/psbmpc.cpp src/psbmpc_lib/ownship.cpp src/psbmpc_lib/kf.cpp src/psbmpc_lib/mrou.cpp src/psbmpc_lib/cpe.cpp  src/psbmpc_lib/obstacle.cpp)


message("Your C++ compiler supports these C++ features:")
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
  message("${i}")
endforeach()

