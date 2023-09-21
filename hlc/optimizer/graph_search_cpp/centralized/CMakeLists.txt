add_library(${PROJECT_NAME}_centralized INTERFACE)
target_include_directories(${PROJECT_NAME}_centralized INTERFACE include)
target_link_libraries(${PROJECT_NAME}_centralized INTERFACE ${PROJECT_NAME}_common collision boost_geometry boost_heap node utility)
target_compile_features(${PROJECT_NAME}_centralized INTERFACE cxx_std_20)