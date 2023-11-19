#include "towable_local_planner/towable_local_planner.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#include <geometry_msgs/PoseArray.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

#include <agv_control_msgs/TowableLocalPlannerFeedback.h>

PLUGINLIB_EXPORT_CLASS(towable_local_planner::TowableLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace towable_local_planner
{

TowableLocalPlanner::TowableLocalPlanner() :
  move_base_nh_("~"),
  tf_(new tf2_ros::Buffer),
  initialized_(false)
{}

TowableLocalPlanner::TowableLocalPlanner(std::string name, tf2_ros::Buffer* tf,
                                         costmap_2d::Costmap2DROS* costmap_ros) :
  move_base_nh_("~"),
  tf_(tf),
  initialized_(false)
{
  initialize(name, tf, costmap_ros);
}

TowableLocalPlanner::~TowableLocalPlanner()
{}

void TowableLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                     costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!isInitialized())
  {
    ros::NodeHandle nh(""), pnh("~/" + name);

    global_plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
    local_plan_pub_ = pnh.advertise<nav_msgs::Path>("local_plan", 1);
    robot_trajectory_pub_ = pnh.advertise<nav_msgs::Path>("robot_trajectory", 1);
    cart_trajectory_pub_ = pnh.advertise<nav_msgs::Path>("cart_trajectory", 1);
    pose_array_pub_ = pnh.advertise<geometry_msgs::PoseArray>("poses", 1);
    feedback_pub_ = pnh.advertise<agv_control_msgs::TowableLocalPlannerFeedback>("feedback", 10);
    cart_yaw_sub_ = nh.subscribe("cart_yaw", 10, &TowableLocalPlanner::cartYawCB, this);

    pnh.param("vel_x_forward", v_forward_, 0.1);
    pnh.param("vel_x_backward", v_backward_, 0.1);
    pnh.param("vel_theta_max", omega_max_, 0.5);

    pnh.param("distance_error_weight_1_f",
      q_e_.at(FollowingMode::FORWARD).at(Target::ROBOT), 1.0);
    pnh.param("yaw_error_weight_1_f",
      q_theta_.at(FollowingMode::FORWARD).at(Target::ROBOT), 0.2);
    pnh.param("distance_error_weight_2_f",
      q_e_.at(FollowingMode::FORWARD).at(Target::CART), 0.2);
    pnh.param("yaw_error_weight_2_f",
      q_theta_.at(FollowingMode::FORWARD).at(Target::CART), 0.2);
    pnh.param("distance_error_weight_1_b",
      q_e_.at(FollowingMode::BACKWARD).at(Target::ROBOT), 0.6);
    pnh.param("yaw_error_weight_1_b",
      q_theta_.at(FollowingMode::BACKWARD).at(Target::ROBOT), 0.2);
    pnh.param("distance_error_weight_2_b",
      q_e_.at(FollowingMode::BACKWARD).at(Target::CART), 1.0);
    pnh.param("yaw_error_weight_2_b",
      q_theta_.at(FollowingMode::BACKWARD).at(Target::CART), 0.6);
    pnh.param("rot_vel_weight", r_, 0.2);
    pnh.param("rot_vel_var_weight", r_d_, 0.2);
    pnh.param("prediction_horizon", t_p_, 3.0);

    pnh.param("towing", is_towing_, true);
    pnh.param("global_frame", global_frame_, std::string(""));
    pnh.param("robot_base_frame", robot_base_frame_, std::string(""));
    pnh.param("cart_base_frame", cart_base_frame_, std::string(""));
    pnh.param("coupler_frame", coupler_frame_, std::string(""));

    move_base_nh_.param("global_costmap/global_frame",
                        global_frame_, std::string(""));
    move_base_nh_.param("global_costmap/robot_base_frame",
                        robot_base_frame_, std::string(""));
    move_base_nh_.param("controller_frequency", controller_frequency_, 20.0);
    move_base_nh_.setParam("recovery_behavior_enabled", false);

    dsrv_ = new dynamic_reconfigure::Server<TowableLocalPlannerConfig>(pnh);
    dsrv_->setCallback(boost::bind(&TowableLocalPlanner::reconfigureCB, this, _1, _2));

    th_.setGlobalFrame(global_frame_);
    global_plan_.header.frame_id = global_frame_;
    robot_trajectory_.header.frame_id = global_frame_;
    cart_trajectory_.header.frame_id = global_frame_;

    if(is_towing_)
    {
      if(!th_.getOriginDistance(robot_base_frame_, coupler_frame_,
                                l_1_, ros::Duration(5.0)))
      {
        ROS_ERROR("Could not set l_1");
      }
      if(!th_.getOriginDistance(coupler_frame_, cart_base_frame_,
                                l_2_, ros::Duration(5.0)))
      {
        ROS_ERROR("Could not set l_2");
      }
      ROS_INFO("l_1: %5lf, l_2: %5lf", l_1_, l_2_);
    }

    initialized_ = true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing");
  }
}

bool TowableLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  if(!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  global_plan_.header.stamp = ros::Time::now();
  global_plan_.poses.clear();
  global_plan_.poses = global_plan;

  robot_trajectory_.poses.clear();
  cart_trajectory_.poses.clear();

  ph_.setPath(global_plan_);

  reached_goal_ = false;

  return true;
}

bool TowableLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if(!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  if(global_plan_.poses.empty())
  {
    ROS_ERROR("Global plan not set");
    return false;
  }

  if(!th_.getGlobalPose(robot_base_frame_, target_pose_.at(Target::ROBOT)))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if(((following_mode_ == FollowingMode::FORWARD || !is_towing_) &&
         (ph_.getReferencePoint(target_pose_.at(Target::ROBOT)) == ph_.getEnd())) ||
       ((following_mode_ == FollowingMode::BACKWARD) &&
         (ph_.getReferencePoint(target_pose_.at(Target::CART)) == ph_.getEnd())))
  {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    reached_goal_ = true;
    ROS_INFO("Reached Goal");
  }
  else
  {
    if(is_towing_)
    {
      if(!th_.getGlobalPose(cart_base_frame_, target_pose_.at(Target::CART)))
      {
        ROS_ERROR("Could not get cart pose");
        return false;
      }
    }
    
    if(!computeVelocityCommandByMPC(cmd_vel))
    {
      ROS_ERROR("Could not compute velocity command");
      return false;
    }
  }

  return true;
}

bool TowableLocalPlanner::computeVelocityCommandByMPC(geometry_msgs::Twist& cmd_vel)
{
  const int NUM_OF_INPUTS = 1;
  const int NUM_OF_STATES = is_towing_ ? 6 : 3;

  static Eigen::MatrixXd A, B, Q, R;
  static Eigen::VectorXd x_0, X_ref, U_ref, X_max, U_max, q, b_l, b_u;

  std::array<Eigen::Vector2d, NUM_OF_TARGETS> target_point, ref_point, arc_orthog_vec;
  std::array<double, NUM_OF_TARGETS> e, theta;
  std::array<double, NUM_OF_TARGETS> kappa, beta;
  double v, psi, delta;

  bool backward = (following_mode_ == FollowingMode::BACKWARD);

  for(int i = 0; i < (is_towing_ ? 2 : 1); i++)
  {
    target_point.at(i) << target_pose_.at(i).pose.position.x,
                          target_pose_.at(i).pose.position.y;
    auto ref_point_itr = ph_.getReferencePoint(target_pose_.at(i));
    ref_point.at(i) << (*ref_point_itr).pose.position.x,
                        (*ref_point_itr).pose.position.y;

    kappa.at(i) = ph_.getCurvature(ref_point_itr, 0.5);
    beta.at(i) = ph_.getTangentYaw(ref_point_itr);
    if(backward) beta.at(i) -= std::copysign(M_PI, beta.at(i));
    
    arc_orthog_vec.at(i) << std::cos(beta.at(i)+M_PI_2),
                            std::sin(beta.at(i)+M_PI_2);
    
    e.at(i) = (target_point.at(i)-ref_point.at(i)).dot(arc_orthog_vec.at(i));
    theta.at(i) = tf2::getYaw(target_pose_.at(i).pose.orientation) - beta.at(i);
    if(std::abs(theta.at(i)) >= M_PI) theta.at(i) -= std::copysign(2*M_PI, theta.at(i));
  }
  
  v = backward ? -v_backward_ : v_forward_;
  if(is_towing_)
  {
    psi = beta.at(Target::CART) - beta.at(Target::ROBOT);
    delta = phi_ - psi;
  }

  // MPC

  move_base_nh_.param("controller_frequency", controller_frequency_, 20.0);
  double dt = 1/controller_frequency_;
  int num_of_samples = std::round(t_p_/dt);
  int X_size = NUM_OF_STATES*(num_of_samples+1); // index: 0 ~ n
  int U_size = NUM_OF_INPUTS*num_of_samples; // index: 0 ~ n-1

  double e_1 = e.at(Target::ROBOT), e_2 = e.at(Target::CART);
  double theta_1 = theta.at(Target::ROBOT), theta_2 = theta.at(Target::CART);
  double kappa_1 = kappa.at(Target::ROBOT), kappa_2 = kappa.at(Target::CART);

  A.resize(NUM_OF_STATES, NUM_OF_STATES);
  B.resize(NUM_OF_STATES, NUM_OF_INPUTS);
  x_0.resize(NUM_OF_STATES);

  if(is_towing_)
  {
    x_0 << e_1,
           theta_1,
           v,
           e_2,
           theta_2,
           delta;
  
    X_ref = (Eigen::VectorXd(NUM_OF_STATES) << 0,
                                               0,
                                               v,
                                               0,
                                               0,
                                               0)
            .finished().replicate(num_of_samples+1, 1);
    
    A << 1, v*dt, 0,                                  0, 0,                  0,
         0, 1,    -kappa_1*dt,                        0, 0,                  0,
         0, 0,    1,                                  0, 0,                  0,
         0, 0,    0,                                  1, v*std::cos(psi)*dt, 0,
         0, 0,    (-kappa_2*std::cos(psi))*dt,        0, 1,                  kappa_2*v*std::sin(psi)*dt,
         0, 0,    (kappa_1-kappa_2*std::cos(psi))*dt, 0, 0,                  1+kappa_2*v*std::sin(psi)*dt;
    B << 0,
         dt,
         0,
         0,
         -l_1_/l_2_*std::cos(psi)*dt,
         -(1+l_1_/l_2_*std::cos(psi))*dt;

    Q = (Eigen::VectorXd(NUM_OF_STATES) << q_e_.at(following_mode_).at(Target::ROBOT)*10,
                                           q_theta_.at(following_mode_).at(Target::ROBOT),
                                           0,
                                           q_e_.at(following_mode_).at(Target::CART)*10,
                                           q_theta_.at(following_mode_).at(Target::CART),
                                           0)
        .finished().replicate(num_of_samples+1, 1).asDiagonal();
  }
  else
  {
    x_0 << e_1,
           theta_1,
           v;
  
    X_ref = (Eigen::VectorXd(NUM_OF_STATES) << 0,
                                               0,
                                               v)
            .finished().replicate(num_of_samples+1, 1);

    A << 1, v*dt, 0,
         0, 1,    -kappa_1*dt,
         0, 0,    1;
    B << 0,
         dt,
         0;
    
    Q = (Eigen::VectorXd(NUM_OF_STATES) << q_e_.at(following_mode_).at(Target::ROBOT)*10,
                                           q_theta_.at(following_mode_).at(Target::ROBOT),
                                           0)
        .finished().replicate(num_of_samples+1, 1).asDiagonal();
  }

  X_max = Eigen::VectorXd::Constant(X_size, INFINITY);
  U_max = Eigen::VectorXd::Constant(U_size, omega_max_);
  U_ref = Eigen::VectorXd::Constant(U_size, 0);

  R = Eigen::VectorXd::Constant(U_size, r_+2*r_d_).asDiagonal();
  R.diagonal(1) = R.diagonal(-1) = Eigen::VectorXd::Constant(R.rows()-1, -r_d_);
  R(0, 0) = R(R.rows()-1, R.cols()-1) = r_+r_d_;

  Eigen::SparseMatrix<double> P(Q.rows()+R.rows(), Q.cols()+R.cols());
  for(int i = 0; i < Q.rows(); i++)
    for(int j = 0; j < Q.cols(); j++)
      P.insert(i, j) = 2*Q(i, j);
  for(int i = 0; i < R.rows(); i++)
    for(int j = 0; j < R.cols(); j++)
      P.insert(i+Q.rows(), j+Q.cols()) = 2*R(i, j);

  q = -P*((Eigen::VectorXd(X_ref.size()+U_ref.size()) << X_ref, U_ref).finished());

  Eigen::SparseMatrix<double> A_c(2*X_size+U_size, X_size+U_size);
  for(int i = 0; i < X_size; i++)
  {
    A_c.insert(i, i) = -1;
    A_c.insert(i+X_size, i) = 1;
  }
  for(int i = 0; i < U_size; i++)
  {
    for(int j = 0; j < NUM_OF_STATES; j++)
    {
      for(int k = 0; k < NUM_OF_STATES; k++)
        A_c.insert((i+1)*NUM_OF_STATES+j, i*NUM_OF_STATES+k) = A(j, k);
      for(int k = 0; k < NUM_OF_INPUTS; k++)
        A_c.insert((i+1)*NUM_OF_STATES+j, X_size+i*NUM_OF_INPUTS+k) = B(j, k);
    }
    A_c.insert(i+2*X_size, i+X_size) = 1;
  }

  b_l = Eigen::VectorXd::Zero(A_c.rows());
  b_u = Eigen::VectorXd::Zero(A_c.rows());
  b_l.topRows(x_0.rows()) = b_u.topRows(x_0.rows()) = -x_0;
  b_l.middleRows(X_size, X_max.rows())   = -X_max;
  b_l.middleRows(2*X_size, U_max.rows()) = -U_max;
  b_u.middleRows(X_size, X_max.rows())   = X_max;
  b_u.middleRows(2*X_size, U_max.rows()) = U_max;

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfVariables(X_size+U_size);
  solver.data()->setNumberOfConstraints(2*X_size+U_size);
  if(!solver.data()->setHessianMatrix(P)             ||
     !solver.data()->setGradient(q)                  ||
     !solver.data()->setLinearConstraintsMatrix(A_c) ||
     !solver.data()->setLowerBound(b_l)              ||
     !solver.data()->setUpperBound(b_u))
  {
    ROS_ERROR("Could not set MPC parameters");
    return false;
  }

  if(!solver.initSolver())
  {
    ROS_ERROR("Could not initialize MPC solver");
    return false;
  }

  if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
  {
    ROS_ERROR("Could not get solution from MPC solver");
    return false;
  }

  double omega = solver.getSolution()(X_size);

  cmd_vel.angular.z = omega;
  cmd_vel.linear.x = v;

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = global_frame_;
  pose_array.header.stamp = ros::Time::now();
  pose_array.poses.resize(1*NUM_OF_TARGETS);
  rh_.clearCircle("arc");

  for(int i = 0; i < (is_towing_ ? 2 : 1); i++)
  {
    geometry_msgs::Pose pose;
    tf2::Quaternion quat;
    pose.position.x = ref_point.at(i).x();
    pose.position.y = ref_point.at(i).y();
    quat.setRPY(0, 0, beta.at(i));
    pose.orientation = tf2::toMsg(quat);
    pose_array.poses.push_back(pose);

    if(kappa.at(i) != 0)
    {
      double arc_radius = 1/std::abs(kappa.at(i));
      Eigen::Vector2d arc_center =
        ref_point.at(i) + std::copysign(arc_radius, kappa.at(i))*arc_orthog_vec.at(i);
      rh_.drawCircle(global_frame_, "arc", {arc_center.x(), arc_center.y()},
                      arc_radius, {0, 0, 1});
    }
  }
  pose_array_pub_.publish(pose_array);

  global_plan_pub_.publish(global_plan_);

  robot_trajectory_.header.stamp = ros::Time::now();
  robot_trajectory_.poses.push_back(target_pose_.at(Target::ROBOT));
  robot_trajectory_pub_.publish(robot_trajectory_);

  if(is_towing_)
  {
    cart_trajectory_.header.stamp = ros::Time::now();
    cart_trajectory_.poses.push_back(target_pose_.at(Target::CART));
    cart_trajectory_pub_.publish(cart_trajectory_);
  }

  agv_control_msgs::TowableLocalPlannerFeedback feedback;
  feedback.stamp = ros::Time::now();
  feedback.e_1 = e_1;
  feedback.theta_1 = theta_1;
  feedback.kappa_1 = kappa_1;
  feedback.beta_1 = beta.at(0);
  if(is_towing_)
  {
    feedback.e_2 = e_2;
    feedback.theta_2 = theta_2;
    feedback.kappa_2 = kappa_2;
    feedback.beta_2 = beta.at(1);
    feedback.psi = psi;
    feedback.delta = delta;
  }
  feedback.cmd_vel = cmd_vel;

  feedback_pub_.publish(feedback);

  return true;
}

void TowableLocalPlanner::cartYawCB(const std_msgs::Float32 cart_yaw)
{
  phi_ = cart_yaw.data;
}

void TowableLocalPlanner::reconfigureCB(towable_local_planner::TowableLocalPlannerConfig& config,
                                        uint32_t level)
{
  v_forward_ = config.vel_x_forward;
  v_backward_ = config.vel_x_backward;
  omega_max_ = config.vel_theta_max;
  following_mode_ = config.following_mode;

  q_e_.at(FollowingMode::FORWARD).at(Target::ROBOT) = config.distance_error_weight_1_f;
  q_theta_.at(FollowingMode::FORWARD).at(Target::ROBOT) = config.yaw_error_weight_1_f;
  q_e_.at(FollowingMode::FORWARD).at(Target::CART) = config.distance_error_weight_2_f;
  q_theta_.at(FollowingMode::FORWARD).at(Target::CART) = config.yaw_error_weight_2_f;
  q_e_.at(FollowingMode::BACKWARD).at(Target::ROBOT) = config.distance_error_weight_1_b;
  q_theta_.at(FollowingMode::BACKWARD).at(Target::ROBOT) = config.yaw_error_weight_1_b;
  q_e_.at(FollowingMode::BACKWARD).at(Target::CART) = config.distance_error_weight_2_b;
  q_theta_.at(FollowingMode::BACKWARD).at(Target::CART) = config.yaw_error_weight_2_b;
  r_ = config.rot_vel_weight;
  r_d_ = config.rot_vel_var_weight;
  t_p_ = config.prediction_horizon;
}

} // namespace towable_local_planner