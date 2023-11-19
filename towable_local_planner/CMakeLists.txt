cmake_minimum_required(VERSION 3.0.2)
project(towable_local_planner)
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  agv_control_lib
  agv_control_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES towable_local_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    rospy
    std_msgs
    tf2
    tf2_ros
    agv_control_lib
    agv_control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

add_library(towable_local_planner
  src/towable_local_planner.cpp
)
add_dependencies(towable_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(towable_local_planner
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

install(TARGETS towable_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES towable_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)