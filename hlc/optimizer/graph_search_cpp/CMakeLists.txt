cmake_minimum_required(VERSION 3.19)

project(graph_search_cpp)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(CPM_DOWNLOAD_VERSION 0.38.1)

if (CPM_SOURCE_CACHE)
    set(CPM_DOWNLOAD_LOCATION "${CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
elseif (DEFINED ENV{CPM_SOURCE_CACHE})
    set(CPM_DOWNLOAD_LOCATION "$ENV{CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
else ()
    set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
endif ()

# Expand relative path. This is important if the provided path contains a tilde (~)
get_filename_component(CPM_DOWNLOAD_LOCATION ${CPM_DOWNLOAD_LOCATION} ABSOLUTE)

function(download_cpm)
    message(STATUS "Downloading CPM.cmake to ${CPM_DOWNLOAD_LOCATION}")
    file(DOWNLOAD
            https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake
            ${CPM_DOWNLOAD_LOCATION}
    )
endfunction()

if (NOT (EXISTS ${CPM_DOWNLOAD_LOCATION}))
    download_cpm()
else ()
    # resume download if it previously failed
    file(READ ${CPM_DOWNLOAD_LOCATION} check)
    if ("${check}" STREQUAL "")
        download_cpm()
    endif ()
endif ()

include(${CPM_DOWNLOAD_LOCATION})

if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11.1)
    message("using std::barrier backport!")
    CPMAddPackage(
            NAME cxx20_concurrency_lib
            GITHUB_REPOSITORY "xentara/cxx20-concurrency"
            GIT_TAG "main"
    )
endif ()

#add_subdirectory(matplotlib)
add_subdirectory(boost_cpm)
add_subdirectory(collision)
add_subdirectory(utility)
add_subdirectory(node)
add_subdirectory(matlab_utility)
add_subdirectory(common)
add_subdirectory(centralized)
add_subdirectory(priority)
add_subdirectory(mex)