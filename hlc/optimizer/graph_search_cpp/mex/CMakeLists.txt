if (APPLE)
    set(Matlab_ROOT_DIR /Applications/MATLAB_R2023b.app)
    find_package(Matlab REQUIRED)

    # Set DYLD_LIBRARY_PATH for execution
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/runtime/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/bin/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/sys/os/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/extern/bin/maci64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/bin/maci64)
elseif (WIN32)
    find_package(Matlab REQUIRED)
    # Set PATH for execution
    set(Path ${Path}:${Matlab_ROOT_DIR}/runtime/win64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/bin/win64)
else ()
    find_package(Matlab REQUIRED)
    # Set LD_LIBRARY_PATH for execution
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/runtime/glnxa64)
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/bin/glnxa64)
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/sys/os/glnxa64)
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/extern/bin/glnxa64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/bin/glnxa64)
endif ()

if (NOT MSVC)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
endif ()

if (MSVC)
    add_compile_options(/bigobj)
endif ()

include_directories(include)
matlab_add_mex(NAME ${PROJECT_NAME}_centralized_mex SRC src/centralized_main.cpp)
target_link_libraries(${PROJECT_NAME}_centralized_mex ${PROJECT_NAME}_centralized ${PROJECT_NAME}_common boost_geometry boost_heap matlab_utility)
target_compile_features(${PROJECT_NAME}_centralized_mex PUBLIC cxx_std_20)

matlab_add_mex(NAME ${PROJECT_NAME}_priority_mex SRC src/priority_main.cpp)
target_link_libraries(${PROJECT_NAME}_priority_mex ${PROJECT_NAME}_priority ${PROJECT_NAME}_common boost_geometry boost_heap matlab_utility)
target_compile_features(${PROJECT_NAME}_priority_mex PUBLIC cxx_std_20)

if (MSVC)
    set_target_properties(${PROJECT_NAME}_centralized_mex PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}
            RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}
    )
    set_target_properties(${PROJECT_NAME}_priority_mex PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}
            RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}
    )
endif ()