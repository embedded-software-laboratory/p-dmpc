add_library(collision src/CollisionDetection.cpp src/SAT.cpp)
target_include_directories(collision PUBLIC include)
target_link_libraries(collision PUBLIC)
target_compile_features(collision PUBLIC cxx_std_17)

#add_executable(collision_test test/test.cpp)
#target_link_libraries(collision_test PUBLIC collision matplotlib_cpp)