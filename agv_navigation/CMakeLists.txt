cmake_minimum_required(VERSION 3.0.2)
project(agv_navigation)
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    rviz
    std_msgs
    tf2
    agv_control_msgs
    jsk_rviz_plugins
    laser_line_extraction
    agv_control_lib
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets  
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

set(OSQPEIGEN_LIBRARIES OsqpEigen::OsqpEigen)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation_control_panel
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    rviz
    std_msgs
    tf2
    agv_control_msgs
    jsk_rviz_plugins
    laser_line_extraction
    agv_control_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

add_executable(cart_yaw_publisher
  src/cart_yaw_publisher.cpp
)
target_link_libraries(cart_yaw_publisher
  ${catkin_LIBRARIES}
)

add_executable(jackknife_prevention
  src/jackknife_prevention.cpp
)
add_dependencies(jackknife_prevention ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jackknife_prevention
  ${catkin_LIBRARIES}
)

add_executable(path_following_controller
  src/path_following_controller.cpp
)
add_dependencies(path_following_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_following_controller
  ${catkin_LIBRARIES}
  ${OSQPEIGEN_LIBRARIES}
)

add_executable(path_generator
  src/path_generator.cpp
)
add_dependencies(path_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_generator
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_library(navigation_control_panel
  include/agv_navigation/navigation_control_panel.h
  src/navigation_control_panel.cpp
)
target_link_libraries(navigation_control_panel
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)