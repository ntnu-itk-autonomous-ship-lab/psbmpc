
#find_package(PythonLibs REQUIRED)
#find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

#add_executable(tester test_ownship.cpp)
#add_executable(tester test_obstacle.cpp)
add_executable(tester test_cpe.cpp)
#add_executable(tester test_mrou.cpp)
#add_executable(tester test_kf.cpp)
#add_executable(tester test_psbmpc.cpp)
#add_executable(tester test_prng.cpp)

target_include_directories(tester PUBLIC
	${CMAKE_SOURCE_DIR}/include
	)



target_link_libraries(tester psbmpc_lib) # ${Matlab_LIBRARIES})

target_compile_features(tester PUBLIC cxx_std_11)


