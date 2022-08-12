cmake_minimum_required(VERSION 2.8.3)
project(metric_evaluation)

SET(CMAKE_BUILD_TYPE Release)  # Debug, Release
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        people_msgs_utils
        move_base_benchmark
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${move_base_benchmark_INCLUDE_DIRS}
    ${people_msgs_utils_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    metrics.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${move_base_benchmark_LIBRARIES}
)
