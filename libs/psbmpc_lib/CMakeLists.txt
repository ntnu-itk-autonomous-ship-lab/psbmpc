find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)

find_package(CUDA COMPONENTS THRUST REQUIRED)

add_library(psbmpc_lib STATIC 
	src/psbmpc.cpp 
	src/ownship.cpp 
	src/cpe.cpp 
	src/kf.cpp 
	src/obstacle.cpp 
	src/obstacle_ship.cpp 
	src/obstacle_sbmpc.cpp 
	src/prediction_obstacle.cpp 
	src/mrou.cpp 
	src/utilities.cpp)

message("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")

target_include_directories(psbmpc_lib PUBLIC 
	include
	${CMAKE_SOURCE_DIR}/libs/third_party_libs
	${Matlab_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	)

message("CUDA_LIBARIES: ${CUDA_LIBRARIES}")

target_link_libraries(psbmpc_lib ${Matlab_LIBRARIES} ${CUDA_LIBRARIES})

target_compile_features(psbmpc_lib PUBLIC cxx_std_11)


