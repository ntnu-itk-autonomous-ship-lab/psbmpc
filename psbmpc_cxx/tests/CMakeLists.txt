cmake_minimum_required(VERSION 3.17.3)

include(FetchContent)
FetchContent_Declare(
googletest
# Googletest commit to use. Should be updated regularly.
URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
FetchContent_MakeAvailable(googletest)
enable_testing()

set(CPP_TEST_SOURCES
  test_kf.cpp
  test_prng.cpp
  test_mrou.cpp
  test_ownship.cpp
  test_obstacle.cpp
  test_obstacle_ship.cpp
  test_obstacle_predictor.cpp
  test_cpe_cpu.cpp
  test_psbmpc_cost_class.cpp
  test_read_shapefile.cpp
)

add_executable(${PROJECT_NAME}_cpu_tests ${CPP_TEST_SOURCES})
target_link_libraries(${PROJECT_NAME}_cpu_tests PRIVATE PSBMPC gtest_main)
add_test(NAME ${PROJECT_NAME}_cpu_tests COMMAND ${PROJECT_NAME}_cpu_tests)

function(add_cuda_test TEST_NAME SOURCE_FILE)
  add_executable(${TEST_NAME} ${SOURCE_FILE})
  target_link_libraries(${TEST_NAME} PRIVATE PSBMPC gtest_main)
  set_target_properties(${TEST_NAME} PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

if (USE_GPU_PSBMPC)
  add_cuda_test(test_cpe_gpu test_cpe_gpu.cu)
  add_cuda_test(test_thrust test_thrust.cu)
  add_cuda_test(test_tml test_tml.cu)
  add_cuda_test(test_tml_pdmatrix test_tml_pdmatrix.cu)
endif()
