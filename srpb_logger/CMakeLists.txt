cmake_minimum_required(VERSION 3.12)
project(srpb_logger)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        geometry_msgs
        people_msgs
        people_msgs_utils
        tf2
        tf2_geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        roscpp
        geometry_msgs
        people_msgs
        people_msgs_utils
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${people_msgs_utils_INCLUDE_DIRS}
)

# move_base
add_library(${PROJECT_NAME}_lib
  include/${PROJECT_NAME}/benchmark_logger.h
  include/${PROJECT_NAME}/people_logger.h
  include/${PROJECT_NAME}/robot_data.h
  include/${PROJECT_NAME}/robot_logger.h
  src/people_logger.cpp
  src/robot_logger.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${people_msgs_utils_LIBRARIES}
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

## Testing
# catkin run_tests --no-deps srpb_logger --verbose
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_people_logger test/test_people_logger.cpp)
  if(TARGET test_people_logger)
    target_link_libraries(test_people_logger ${PROJECT_NAME}_lib)
  endif()
endif()
