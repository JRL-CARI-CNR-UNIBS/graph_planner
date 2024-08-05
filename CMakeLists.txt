cmake_minimum_required(VERSION 3.16.0)
project(graph_planner)
add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  graph_display
)


find_package(graph_core REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_planner
  CATKIN_DEPENDS  moveit_core moveit_ros_planning roscpp graph_display
  DEPENDS graph_core
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/graph_planner.cpp
  src/${PROJECT_NAME}/graph_planner_manager.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
PUBLIC
  ${catkin_LIBRARIES}
  graph_core::graph_core
)



#############
## Install ##
#############

 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 install(FILES
   plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dirrt_star.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
