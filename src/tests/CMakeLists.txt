

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

target_compile_features(tester PRIVATE cxx_std_11)

target_link_libraries(tester psbmpc_lib)