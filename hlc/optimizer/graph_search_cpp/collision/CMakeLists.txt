add_library(collision src/CollisionDetection.cpp src/SAT.cpp)
target_include_directories(collision PUBLIC include)
target_link_libraries(collision PUBLIC)
target_compile_features(collision PUBLIC cxx_std_17)