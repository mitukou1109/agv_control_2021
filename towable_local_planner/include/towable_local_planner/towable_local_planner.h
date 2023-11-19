#ifndef TOWABLE_LOCAL_PLANNER_TOWABLE_LOCAL_PLANNER_H_
#define TOWABLE_LOCAL_PLANNER_TOWABLE_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>

#include <agv_control_msgs/TowableLocalPlannerConfig.h>
#include <agv_control_lib/path_handler.h>
#include <agv_control_lib/rviz_handler.h>
#include <agv_control_lib/tf_handler.h>

namespace towable_local_planner
{

class TowableLocalPlanner : public nav_core::BaseLocalPlanner
{
public:

  TowableLocalPlanner();

  TowableLocalPlanner(std::string name, tf2_ros::Buffer* tf,
                      costmap_2d::Costmap2DROS* costmap_ros);

  ~TowableLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached() { return reached_goal_; }

  bool isInitialized() { return initialized_; }

private:

  enum FollowingMode
  {
    FORWARD,
    BACKWARD
  };

  enum Target
  {
    ROBOT,
    CART
  };

  static constexpr int NUM_OF_TARGETS = 2; // AGV & Trailer

  void reconfigureCB(towable_local_planner::TowableLocalPlannerConfig& config, uint32_t level);

  bool computeVelocityCommandByMPC(geometry_msgs::Twist& cmd_vel);

  void cartYawCB(const std_msgs::Float32 cart_yaw);

  tf2_ros::Buffer* tf_;
  std::string global_frame_;
  std::string robot_base_frame_;

  bool reached_goal_;

  ros::Publisher global_plan_pub_, local_plan_pub_;

  bool initialized_;

  
  ros::NodeHandle move_base_nh_;
  ros::Publisher robot_trajectory_pub_, cart_trajectory_pub_,
                 pose_array_pub_, feedback_pub_;
  ros::Subscriber cart_yaw_sub_;

  dynamic_reconfigure::Server<towable_local_planner::TowableLocalPlannerConfig>* dsrv_;
  
  TFHandler th_;
  std::string coupler_frame_, cart_base_frame_;
  std::array<geometry_msgs::PoseStamped, NUM_OF_TARGETS> target_pose_;

  PathHandler ph_;
  nav_msgs::Path global_plan_, robot_trajectory_, cart_trajectory_;

  RvizHandler rh_;

  bool is_towing_;
  float phi_;
  double l_1_, l_2_;

  double v_forward_, v_backward_, omega_max_;

  int following_mode_;

  std::array<std::array<double, NUM_OF_TARGETS>, 2> q_e_, q_theta_;
  double r_, r_d_;
  double controller_frequency_, t_p_;
};

} // namespace towable_local_planner

#endif // TOWABLE_LOCAL_PLANNER_TOWABLE_LOCAL_PLANNER_H_