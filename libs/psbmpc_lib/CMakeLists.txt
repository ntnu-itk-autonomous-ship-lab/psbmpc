add_library(psbmpc_lib STATIC src/psbmpc.cpp src/ownship.cpp src/cpe.cpp src/kf.cpp src/obstacle.cpp src/mrou.cpp)
target_include_directories(psbmpc_lib PRIVATE
	include
	${CMAKE_SOURCE_DIR}/libs/third_party_libs
	)

target_compile_features(psbmpc_lib PUBLIC cxx_std_11)
