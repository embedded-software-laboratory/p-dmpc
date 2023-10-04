add_library(${PROJECT_NAME}_priority INTERFACE)
target_include_directories(${PROJECT_NAME}_priority INTERFACE include)

if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11.1)
    target_link_libraries(${PROJECT_NAME}_priority INTERFACE matlab_utility ${PROJECT_NAME}_common collision boost_geometry boost_heap node utility c++20-concurrency)
else ()
    target_link_libraries(${PROJECT_NAME}_priority INTERFACE matlab_utility ${PROJECT_NAME}_common collision boost_geometry boost_heap node utility)
endif ()

target_compile_features(${PROJECT_NAME}_priority INTERFACE cxx_std_20)