add_executable(tester test.cpp)

target_link_libraries(tester psbmpc_lib)

target_compile_features(tester PUBLIC cxx_std_11)


