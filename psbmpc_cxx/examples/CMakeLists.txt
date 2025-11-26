cmake_minimum_required(VERSION 3.17.3)

add_executable(demo_psbmpc run_psbmpc.cpp)
add_executable(demo_psbmpc_grounding run_psbmpc_with_grounding_hazards.cpp)
add_executable(demo_psbmpc_new_cost run_psbmpc_new_cost.cpp)
target_link_libraries(demo_psbmpc PRIVATE PSBMPC)
target_link_libraries(demo_psbmpc_grounding PRIVATE PSBMPC)
target_link_libraries(demo_psbmpc_new_cost PRIVATE PSBMPC)

if (USE_GPU_PSBMPC)
  set_target_properties(demo_psbmpc PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  set_target_properties(demo_psbmpc_grounding PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  set_target_properties(demo_psbmpc_new_cost PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
endif()
