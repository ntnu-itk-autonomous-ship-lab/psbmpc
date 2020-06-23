
#find_package(PythonLibs REQUIRED)
find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

add_executable(tester test_ownship.cpp)
target_include_directories(tester PUBLIC
	${Matlab_INCLUDE_DIRS}
	)

message("Matlab include dirs: ${Matlab_INCLUDE_DIRS}")


target_link_libraries(tester psbmpc_lib ${Matlab_LIBRARIES})
message("Matlab libraries: ${Matlab_LIBRARIES}")

target_compile_features(tester PUBLIC cxx_std_11)


