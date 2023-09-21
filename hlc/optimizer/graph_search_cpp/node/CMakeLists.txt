add_library(node INTERFACE)
target_include_directories(
        node INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(node INTERFACE cxx_std_20)
target_link_libraries(node INTERFACE utility)
