cmake_minimum_required(VERSION 3.19)

project(optimizer)

set(CMAKE_CXX_STANDARD 23)

if (APPLE)
    set(CMAKE_OSX_ARCHITECTURES "x86_64")
    set(Matlab_ROOT_DIR /Applications/MATLAB_R2022b.app)

    # Set DYLD_LIBRARY_PATH for execution
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/runtime/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/bin/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/sys/os/maci64)
    set(DYLD_LIBRARY_PATH ${DYLD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/extern/bin/maci64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/extern/bin/maci64)

    # Set mex Function Output to current optimizer Directory
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
elseif (WIN32)
    # Set PATH for execution
    set(Path ${Path}:${Matlab_ROOT_DIR}/runtime/win64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/extern/bin/win64)

else ()
    # Set LD_LIBRARY_PATH for execution
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/runtime/glnxa64)
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/bin/glnxa64)
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/sys/os/glnxa64)
    set(LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${Matlab_ROOT_DIR}/extern/bin/glnxa64)

    # Included directories
    include_directories(${Matlab_ROOT_DIR}/extern/include/)

    # Add linker search directories
    link_directories(${Matlab_ROOT_DIR}/extern/bin/glnxa64)

    # Set mex Function Output to current optimizer Directory
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/mex)
endif ()

if (APPLE)
    find_package(Boost REQUIRED PATHS "/usr/local/homebrew/" NO_DEFAULT_PATH)
    find_package(TBB REQUIRED PATHS "/usr/local/homebrew/" NO_DEFAULT_PATH)
else ()
    MESSAGE(HI)
    find_package(Boost REQUIRED)
    find_package(TBB REQUIRED)
endif ()
include_directories(${Boost_INCLUDE_DIRS})

find_package(Matlab) # on M1 Mac use x64 version of cmake to find Matlab ... with brew in /usr/local/homebrew/bin/cmake

include_directories(include)

matlab_add_mex(NAME ${PROJECT_NAME} SRC src/main.cpp)
target_link_libraries(${PROJECT_NAME} TBB::tbb ${Boost_LIBRARIES})

if (WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG ..
            RUNTIME_OUTPUT_DIRECTORY_RELEASE ..
            )
endif ()