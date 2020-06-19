
find_package(PythonLibs REQUIRED)


add_executable(tester test_ownship.cpp)
target_include_directories(tester PRIVATE 
	${PYTHON_INCLUDE_DIRS} 
	)

message("Python include dirs: {PYTHON_INCLUDE_DIRS}")


target_link_libraries(tester psbmpc_lib ${PYTHON_LIBRARIES})

target_compile_features(tester PUBLIC cxx_std_11)


