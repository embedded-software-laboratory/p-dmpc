if (APPLE)
    set(Matlab_ROOT_DIR /Applications/MATLAB_R2023a.app)
    find_package(Matlab REQUIRED)

    # Set DYLD_LIBRARY_PATH for execution
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/runtime/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/bin/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/sys/os/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/extern/bin/maci64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/extern/bin/maci64)
elseif (WIN32)
    # Set PATH for execution
    set(Path ${Path}:${Matlab_ROOT_DIR}/runtime/win64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/extern/bin/win64)
    find_package(Matlab REQUIRED)
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
    link_directories(${Matlab_ROOT_DIR}/extern/bin/glnxa64)
endif ()

if (NOT MSVC)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
endif ()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

add_library(matlab_utility src/ArrayTypePrinting.cpp src/MatlabPrinter.cpp)
target_include_directories(matlab_utility PUBLIC include ${Matlab_ROOT_DIR}/extern/include)
target_link_libraries(matlab_utility PUBLIC Matlab::MatlabDataArray utility)
target_compile_features(matlab_utility PUBLIC cxx_std_17)
set_property(TARGET matlab_utility PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(matlab_utility_test test/test.cpp)
target_link_libraries(matlab_utility_test matlab_utility)
target_compile_features(matlab_utility_test PUBLIC cxx_std_17)

matlab_add_mex(NAME matlab_utility_mex_test SRC test/mex_test.cpp)
target_link_libraries(matlab_utility_mex_test matlab_utility)
target_compile_features(matlab_utility_mex_test PUBLIC cxx_std_17)

if (MSVC)
    set_target_properties(matlab_utility_mex_test PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/build
            RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/build
            )
endif ()