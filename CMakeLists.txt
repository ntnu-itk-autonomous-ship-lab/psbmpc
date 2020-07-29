cmake_minimum_required(VERSION 3.10)
project(psbmpc_test LANGUAGES CXX CUDA)

if (MSVC)
	add_compile_options(/W4)
else()
	add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer -g)
endif()


add_subdirectory(libs/psbmpc_lib_gpu)
add_subdirectory(src/tests)

set(CMAKE_VERBOSE_MAKEFILE ON)


#message("Your C++ compiler supports these C++ features:")
#foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
#  message("${i}")
#endforeach()



