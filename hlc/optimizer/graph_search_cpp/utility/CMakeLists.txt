CPMAddPackage(
        NAME nlohmann_json
        GITHUB_REPOSITORY "nlohmann/json"
        VERSION 3.11.2
)

add_library(utility src/Printer.cpp src/Watchdog.cpp src/MatlabException.cpp)
target_include_directories(utility PUBLIC include)
target_link_libraries(utility PUBLIC nlohmann_json::nlohmann_json)
target_compile_features(utility PUBLIC cxx_std_20)

add_executable(utility_test test/test.cpp)
target_link_libraries(utility_test PUBLIC utility)