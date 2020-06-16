find_package(Eigen REQUIRED)

add_library(psbmpc_lib STATIC psbmpc.h psbmpc.cpp ownship.h ownship.cpp cpe.h cpe.cpp kf.h kf.cpp obstacle.h obstacle.cpp mrou.h mrou.cpp psbmpc_index.h utilities.h)

target_compile_features(psbmpc_lib PUBLIC cxx_std_11)
