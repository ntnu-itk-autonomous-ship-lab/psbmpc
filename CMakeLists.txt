cmake_minimum_required(VERSION 3.10)
project(psbmpc_test)

if (MSVC)
	add_compile_options(/W4)
else()
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(libs/psbmpc_lib)
add_subdirectory(src/tests)

set(CMAKE_VERBOSE_MAKEFILE ON)


#message("Your C++ compiler supports these C++ features:")
#foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
#  message("${i}")
#endforeach()



