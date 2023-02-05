cmake_minimum_required(VERSION 3.12)
project(metric_evaluation)

SET(CMAKE_BUILD_TYPE Release)  # Debug, Release
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        people_msgs_utils
        move_base_benchmark
        angles
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${move_base_benchmark_INCLUDE_DIRS}
    ${people_msgs_utils_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    # main
    src/metric_evaluation.cpp
    # libs
    include/postprocessing/metric.h
    include/postprocessing/metric_gaussian.h
    include/postprocessing/rewinder.h
    include/postprocessing/utils.h
    # specific metrics
    include/postprocessing/metrics/backward_movements.h
    include/postprocessing/metrics/computational_efficiency.h
    include/postprocessing/metrics/cumulative_heading_change.h
    include/postprocessing/metrics/formation_space_instrusion.h
    include/postprocessing/metrics/inplace_rotations.h
    include/postprocessing/metrics/motion_efficiency.h
    include/postprocessing/metrics/obstacle_safety.h
    include/postprocessing/metrics/oscillations.h
    include/postprocessing/metrics/path_linear_length.h
    include/postprocessing/metrics/person_disturbance.h
    include/postprocessing/metrics/personal_space_instrusion.h
    include/postprocessing/metrics/velocity_smoothness.h
    # sources
    src/metric_gaussian.cpp
    src/rewinder.cpp
    src/utils.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${move_base_benchmark_LIBRARIES}
)
