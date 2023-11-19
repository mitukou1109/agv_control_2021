cmake_minimum_required(VERSION 3.0.2)
project(agv_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  std_msgs
)

add_action_files(FILES
  PathFollowing.action
)
add_message_files(FILES
  TowableLocalPlannerFeedback.msg
)
add_service_files(FILES
  OperatePathFollowing.srv
  GeneratePath.srv
  DeletePath.srv
  ShowEntirePath.srv
  ShowTrajectory.srv
)
generate_messages(DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/JackknifePrevention.cfg
  cfg/PathFollowingController.cfg
  cfg/PathGenerator.cfg
  cfg/TowableLocalPlanner.cfg
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    nav_msgs
    rospy
    std_msgs
)