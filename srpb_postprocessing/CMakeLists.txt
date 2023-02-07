cmake_minimum_required(VERSION 3.12)
project(srpb_postprocessing)

SET(CMAKE_BUILD_TYPE Release)  # Debug, Release
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        angles
        geometry_msgs
        people_msgs
        people_msgs_utils
        roscpp
        social_nav_utils
        srpb_logger
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        # none
    CATKIN_DEPENDS
        angles
        geometry_msgs
        people_msgs
        people_msgs_utils
        roscpp
        social_nav_utils
        srpb_logger
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${people_msgs_utils_INCLUDE_DIRS}
    ${social_nav_utils_INCLUDE_DIRS}
    ${srpb_logger_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
    # libs
    include/${PROJECT_NAME}/metric.h
    include/${PROJECT_NAME}/metric_gaussian.h
    include/${PROJECT_NAME}/rewinder.h
    include/${PROJECT_NAME}/utils.h
    # specific metrics
    include/${PROJECT_NAME}/metrics/backward_movements.h
    include/${PROJECT_NAME}/metrics/computational_efficiency.h
    include/${PROJECT_NAME}/metrics/cumulative_heading_change.h
    include/${PROJECT_NAME}/metrics/formation_space_instrusion.h
    include/${PROJECT_NAME}/metrics/inplace_rotations.h
    include/${PROJECT_NAME}/metrics/motion_efficiency.h
    include/${PROJECT_NAME}/metrics/obstacle_safety.h
    include/${PROJECT_NAME}/metrics/oscillations.h
    include/${PROJECT_NAME}/metrics/path_linear_length.h
    include/${PROJECT_NAME}/metrics/person_disturbance.h
    include/${PROJECT_NAME}/metrics/personal_space_instrusion.h
    include/${PROJECT_NAME}/metrics/velocity_smoothness.h
    # sources
    src/metric_gaussian.cpp
    src/rewinder.cpp
    src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${people_msgs_utils_LIBRARIES}
    ${social_nav_utils_LIBRARIES}
    ${srpb_logger_LIBRARIES}
)

add_executable(${PROJECT_NAME}
    # main
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_lib
)
