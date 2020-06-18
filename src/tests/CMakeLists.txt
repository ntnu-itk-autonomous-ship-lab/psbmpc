find_package(Python2 COMPONENTS Development NumPy)



add_executable(tester test_ownship.cpp)

target_link_libraries(tester psbmpc_lib Python2::Python Python2::NumPy)
target_include_directories(tester PRIVATE 
	${Python2_INCLUDE_DIRS} 
	${Python2_NumPy_INCLUDE_DIRS}
	)

target_compile_features(tester PUBLIC cxx_std_11)


