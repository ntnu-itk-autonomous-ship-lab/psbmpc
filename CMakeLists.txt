cmake_minimum_required(VERSION 3.17.3)
project(psbmpc_test LANGUAGES CUDA CXX)

if (MSVC)
	add_compile_options(/W4)
else()
	add_compile_options(-Wall -Wextra -fno-omit-frame-pointer -g)
endif()


add_subdirectory(libs/psbmpc_lib_gpu)
#add_subdirectory(libs/psbmpc_lib)

add_subdirectory(src/tests)

set(CMAKE_VERBOSE_MAKEFILE ON)

target_link_options(tester 
	PRIVATE $<$<LINK_LANGUAGE:CUDA>:-Xlinker -dlink>
)

target_link_libraries(tester 
	PRIVATE psbmpc_lib_gpu 
)
#target_link_libraries(tester psbmpc_lib)


#message("Your C++ compiler supports these C++ features:")
#foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
#  message("${i}")
#endforeach()