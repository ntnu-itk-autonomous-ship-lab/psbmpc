#add_executable(tester test_ownship.cpp)
#add_executable(tester test_obstacle_ship.cpp)
#add_executable(tester test_obstacle.cpp)
#add_executable(tester gpu_implementation_test_files/test_obstacle.cu)
#add_executable(tester test_cpe.cpp)
#add_executable(tester test_mrou.cpp)
#add_executable(tester test_kf.cpp)
#add_executable(tester test_psbmpc.cpp) # for CPU PSBMPC
#add_executable(tester gpu_implementation_test_files/test_psbmpc.cu) # for GPU PSBMPC
#add_executable(tester gpu_implementation_test_files/stack_overflow.cu)
#add_executable(tester test_prng.cpp)
#add_executable(tester gpu_implementation_test_files/test_thrust.cu)
#add_executable(tester gpu_implementation_test_files/test_cml.cu)
add_executable(tester gpu_implementation_test_files/test_cml_pdmatrix.cu)

target_include_directories(tester PRIVATE
	${CMAKE_SOURCE_DIR}/include
	#${CMAKE_SOURCE_DIR}/libs/psbmpc_lib
	${CMAKE_SOURCE_DIR}/libs/psbmpc_lib_gpu
)