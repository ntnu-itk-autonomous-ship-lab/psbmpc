cmake_minimum_required(VERSION 3.17.3)

add_executable(run_psbmpc run_psbmpc.cpp)
add_executable(run_psbmpc_with_grounding_hazards run_psbmpc_with_grounding_hazards.cpp)
add_executable(run_psbmpc_new_cost run_psbmpc_new_cost.cpp)
target_link_libraries(run_psbmpc PRIVATE PSBMPC)
target_link_libraries(run_psbmpc_with_grounding_hazards PRIVATE PSBMPC)
target_link_libraries(run_psbmpc_new_cost PRIVATE PSBMPC)

if (USE_GPU_PSBMPC)
  set_target_properties(run_psbmpc PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  set_target_properties(run_psbmpc_with_grounding_hazards PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  set_target_properties(run_psbmpc_new_cost PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
endif()
