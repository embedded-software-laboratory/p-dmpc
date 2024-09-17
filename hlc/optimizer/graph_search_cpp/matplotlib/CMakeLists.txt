find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pip
        RESULT_VARIABLE EXIT_CODE
        OUTPUT_QUIET
)

if (NOT ${EXIT_CODE} EQUAL 0)
    message(WARNING "No module named pip. It will be installed using the following command: \"${Python3_EXECUTABLE} -m ensurepip --upgrade\".")

    execute_process(
            COMMAND ${Python3_EXECUTABLE} -m ensurepip --upgrade
            RESULT_VARIABLE EXIT_CODE
            OUTPUT_QUIET
    )
endif ()

execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pip show numpy
        RESULT_VARIABLE EXIT_CODE
        OUTPUT_QUIET
)


if (NOT ${EXIT_CODE} EQUAL 0)
    message(WARNING "The \"numpy\" Python3 package is not installed. It will be installed using the following command: \"${Python3_EXECUTABLE} -m pip install numpy\".")
    execute_process(COMMAND ${Python3_EXECUTABLE} -m pip install numpy)
endif ()


execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pip show matplotlib
        RESULT_VARIABLE EXIT_CODE
        OUTPUT_QUIET
)

if (NOT ${EXIT_CODE} EQUAL 0)
    message(WARNING "The \"matplotlib\" Python3 package is not installed. It will be installed using the following command: \"${Python3_EXECUTABLE} -m pip install matplotlib\".")
    execute_process(COMMAND ${Python3_EXECUTABLE} -m pip install matplotlib)
endif ()

add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )

target_link_libraries(matplotlib_cpp INTERFACE
        Python3::Python
        Python3::Module
        )

target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)