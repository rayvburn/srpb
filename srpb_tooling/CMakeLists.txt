cmake_minimum_required(VERSION 3.12)
project(srpb_tooling)

add_definitions(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        costmap_2d
        geometry_msgs
        roscpp
)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED
    COMPONENTS
        opencv_core
        opencv_imgproc
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        geometry_msgs
        roscpp
    DEPENDS
        Ceres
        OpenCV
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  include/${PROJECT_NAME}/obs_dist_calculator.h
  src/obs_dist_calculator.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# install library
install(
    TARGETS ${PROJECT_NAME}_lib
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
