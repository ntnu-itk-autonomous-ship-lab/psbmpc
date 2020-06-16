cmake_minimum_required(version 3.10)
project(psbmpc_test)

if (MSVC)
	add_compile_options(/W4)
else()
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(src/psbmpc_lib)
add_subdirectory(src/test)

add_executable(psbmpc_tester src/test.cpp src/psbmpc_lib/psbmpc.cpp src/psbmpc_lib/ownship.cpp src/psbmpc_lib/kf.cpp src/psbmpc_lib/mrou.cpp src/psbmpc_lib/cpe.cpp  src/psbmpc_lib/obstacle.cpp)


message("Your C++ compiler supports these C++ features:")
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
  message("${i}")
endforeach()

target_compile_features(psbmpc PUBLIC cxx_std_11)

