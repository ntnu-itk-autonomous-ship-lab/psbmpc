

#add_executable(tester test_ownship.cpp)
#add_executable(tester test_obstacle.cpp)
#add_executable(tester test_cpe.cpp)
#add_executable(tester test_mrou.cpp)
#add_executable(tester test_kf.cpp)
add_executable(tester test_psbmpc.cpp)
#add_executable(tester test_prng.cpp)
#add_executable(tester test_thrust.cu)

target_include_directories(tester PUBLIC
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/libs
	)



target_link_libraries(tester psbmpc_lib_gpu)

target_compile_features(tester PUBLIC cxx_std_11)

target_compile_options(tester 
	PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Wno-pedantic -gencode arch=compute_61, code=sm_61 -dc>
	)
