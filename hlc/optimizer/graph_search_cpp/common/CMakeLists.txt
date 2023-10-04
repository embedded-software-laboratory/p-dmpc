add_library(${PROJECT_NAME}_common src/ScenarioType.cpp src/vec2_geometry_operations.cpp src/MPA.cpp src/ConfigData.cpp)
target_include_directories(${PROJECT_NAME}_common PUBLIC include)
target_link_libraries(${PROJECT_NAME}_common PUBLIC collision boost_geometry boost_heap node utility)
target_compile_features(${PROJECT_NAME}_common PUBLIC cxx_std_20)