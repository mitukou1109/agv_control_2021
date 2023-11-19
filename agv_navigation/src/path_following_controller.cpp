#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <ros/ros.h>
#include <OsqpEigen/OsqpEigen.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>

#include <agv_control_msgs/ShowTrajectory.h>
#include <agv_control_msgs/PathFollowingAction.h>
#include <agv_control_msgs/PathFollowingControllerConfig.h>
#include <agv_control_lib/path_handler.h>
#include <agv_control_lib/rviz_handler.h>
#include <agv_control_lib/tf_handler.h>

class PathFollowingController
{
public:

  using PathFollowingActionServer =
    actionlib::SimpleActionServer<agv_control_msgs::PathFollowingAction>;
  using PathFollowingControllerConfigServer =
    dynamic_reconfigure::Server<agv_navigation::PathFollowingControllerConfig>;

  PathFollowingController() :
  nh_(""),
  action_server_(nh_, "path_following", false)
  {
    ros::NodeHandle pnh("~");

    reference_path_pub_ = pnh.advertise<nav_msgs::Path>("reference_path", 1);
    predicted_path_pub_ = pnh.advertise<nav_msgs::Path>("predicted_path", 1);
    robot_trajectory_pub_ = pnh.advertise<nav_msgs::Path>("robot_trajectory", 1);
    cart_trajectory_pub_ = pnh.advertise<nav_msgs::Path>("cart_trajectory", 1);
    pose_array_pub_ = pnh.advertise<geometry_msgs::PoseArray>("poses", 1);
    cmd_vel_pub_ = pnh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    busy_pub_ = pnh.advertise<std_msgs::Bool>("busy", 1);
    cart_yaw_sub_ = nh_.subscribe("cart_yaw", 10,
                                  &PathFollowingController::cartYawCB, this);
    
    show_trajectory_service_server_ =
      pnh.advertiseService("show_trajectory",
                           &PathFollowingController::showTrajectory, this);

    action_server_.registerGoalCallback(
      boost::bind(&PathFollowingController::goalCB, this));
    action_server_.registerPreemptCallback(
      boost::bind(&PathFollowingController::preemptCB, this));

    config_server_.setCallback(
      boost::bind(&PathFollowingController::reconfigureCB, this, _1, _2));

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
    pnh.param("control_period", dt_, 0.1);
    pnh.param("prediction_horizon", t_p_, 3.0);

    pnh.param("towing", is_towing_, true);
    pnh.param("global_frame", global_frame_, std::string(""));
    pnh.param("robot_base_frame", robot_base_frame_, std::string(""));
    pnh.param("cart_base_frame", cart_base_frame_, std::string(""));
    pnh.param("coupler_frame", coupler_frame_, std::string(""));

    th_.setGlobalFrame(global_frame_);
    reference_path_.header.frame_id = global_frame_;
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
    }

    busy_.data = false;

    cmd_vel_timer_ =
      nh_.createTimer(ros::Duration(dt_), &PathFollowingController::publishCmdVel, this,
                      false, false);
    status_timer_ =
      nh_.createTimer(ros::Duration(0.1), &PathFollowingController::publishBusy, this);

    action_server_.start();
  }

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

  // 制御対象となる車輌の数(ここではAGVとトレーラ)
  static constexpr int NUM_OF_TARGETS = 2;

  // 司令速度を計算
  bool computeVelocityCommand(geometry_msgs::Twist& cmd_vel)
  {
    if(reference_path_.poses.empty())
    {
      ROS_ERROR("Reference path not set");
      return false;
    }

    // AGVのグローバル座標を取得
    if(!th_.getGlobalPose(robot_base_frame_, target_pose_.at(Target::ROBOT)))
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    // 前進かつAGVに最も近い経路上の点が経路の終点の場合 or
    // 後退かつトレーラに最も近い経路上の点が経路の始点の場合
    // 経路追従を終了
    if(((following_mode_ == FollowingMode::FORWARD || !is_towing_) &&
         (ph_.getReferencePoint(target_pose_.at(Target::ROBOT)) == ph_.getEnd())) ||
       ((following_mode_ == FollowingMode::BACKWARD) &&
         (ph_.getReferencePoint(target_pose_.at(Target::CART)) == ph_.getEnd())))
    {
      action_server_.setSucceeded();
      busy_.data = false; // 動作状態をスタンバイに
      ROS_INFO("Path following succeeded");
      return false;
    }
    else
    {
      if(is_towing_)
      {
        // 牽引時はトレーラのグローバル座標を取得
        if(!th_.getGlobalPose(cart_base_frame_, target_pose_.at(Target::CART)))
        {
          ROS_ERROR("Could not get cart pose");
          return false;
        }
      }

      // MPCにより司令速度を計算
      if(!computeVelocityCommandByMPC(cmd_vel))
      {
        action_server_.setAborted();
        busy_.data = false; // 動作状態をスタンバイに
        ROS_ERROR("Could not compute velocity command");
        return false;
      }
    }

    return true;
  }

  // MPCにより司令速度を計算
  bool computeVelocityCommandByMPC(geometry_msgs::Twist& cmd_vel)
  {
    const int NUM_OF_INPUTS = 1; // 入力の数
    const int NUM_OF_STATES = is_towing_ ? 6 : 3; // 状態の数

    // MPCに用いる行列・ベクトル
    static Eigen::MatrixXd A, B, Q, R;
    static Eigen::VectorXd x_0, X_ref, U_ref, X_max, U_max, q, b_l, b_u;

    std::array<Eigen::Vector2d, NUM_OF_TARGETS> target_point, ref_point, arc_orthog_vec;
    std::array<double, NUM_OF_TARGETS> e, theta;
    std::array<double, NUM_OF_TARGETS> kappa, beta;
    double v, psi, delta;

    bool backward = (following_mode_ == FollowingMode::BACKWARD);

    // 追従誤差を計算(参考：卒論)
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
    
    // その他の状態を設定
    v = backward ? -v_backward_ : v_forward_;
    if(is_towing_)
    {
      psi = beta.at(Target::CART) - beta.at(Target::ROBOT);
      delta = phi_ - psi;
    }

    // MPCによる最適入力の計算(参考：卒論)

    int num_of_samples = std::round(t_p_/dt_);     // 予測サンプル数
    int X_size = NUM_OF_STATES*(num_of_samples+1); // 添字0〜n
    int U_size = NUM_OF_INPUTS*num_of_samples;     // 添字0〜n-1

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
      
      A << 1, v*dt_, 0,                                   0, 0,                   0,
           0, 1,     -kappa_1*dt_,                        0, 0,                   0,
           0, 0,     1,                                   0, 0,                   0,
           0, 0,     0,                                   1, v*std::cos(psi)*dt_, 0,
           0, 0,     (-kappa_2*std::cos(psi))*dt_,        0, 1,                   kappa_2*v*std::sin(psi)*dt_,
           0, 0,     (kappa_1-kappa_2*std::cos(psi))*dt_, 0, 0,                   1+kappa_2*v*std::sin(psi)*dt_;
      B << 0,
           dt_,
           0,
           0,
           -l_1_/l_2_*std::cos(psi)*dt_,
           -(1+l_1_/l_2_*std::cos(psi))*dt_;

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

      A << 1, v*dt_, 0,
           0, 1,     -kappa_1*dt_,
           0, 0,     1;
      B << 0,
           dt_,
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

    // OSQPの設定
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

    // OSQPを用いて最適化問題を解く
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
      ROS_ERROR("Could not get solution from MPC solver");
      return false;
    }

    // 最適解から入力(旋回角速度)を取り出す
    double omega = solver.getSolution()(X_size);

    // 司令速度を設定
    cmd_vel.angular.z = omega;
    cmd_vel.linear.x = v;

    // 参照点・近似円を表示(デバッグ)
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

    // 参照経路をpub
    reference_path_pub_.publish(reference_path_);

    // AGVの軌跡をpub
    robot_trajectory_.header.stamp = ros::Time::now();
    robot_trajectory_.poses.push_back(target_pose_.at(Target::ROBOT));
    robot_trajectory_pub_.publish(robot_trajectory_);

    // 牽引時はトレーラの軌跡をpub
    if(is_towing_)
    {
      cart_trajectory_.header.stamp = ros::Time::now();
      cart_trajectory_.poses.push_back(target_pose_.at(Target::CART));
      cart_trajectory_pub_.publish(cart_trajectory_);
    }

    // 追従誤差などをフィードバック
    agv_control_msgs::PathFollowingFeedback feedback;
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
    action_server_.publishFeedback(feedback);

    return true;
  }

  // 経路追従を停止
  void stopFollowing()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel_pub_.publish(cmd_vel);

    cmd_vel_timer_.stop();
  }

  // 軌跡を表示
  bool showTrajectory(agv_control_msgs::ShowTrajectoryRequest& req,
                      agv_control_msgs::ShowTrajectoryResponse& res)
  {
    robot_trajectory_pub_.publish(robot_trajectory_);
    res.robot_trajectory = robot_trajectory_;

    cart_trajectory_pub_.publish(cart_trajectory_);
    res.cart_trajectory = cart_trajectory_;

    return true;
  }

  // 司令速度をpub
  void publishCmdVel(const ros::TimerEvent& e)
  {
    geometry_msgs::Twist cmd_vel;

    if(!computeVelocityCommand(cmd_vel) || !action_server_.isActive())
    {
      stopFollowing();
      return;
    }

    cmd_vel_pub_.publish(cmd_vel);
  }

  // 動作状態(busy_)をpub
  void publishBusy(const ros::TimerEvent& e)
  {
    busy_pub_.publish(busy_);
  }

  // 角度差をsub
  void cartYawCB(const std_msgs::Float32 cart_yaw)
  {
    phi_ = cart_yaw.data;
  }

  // PathFollowingアクションのゴールコールバック
  void goalCB()
  {
    auto goal = action_server_.acceptNewGoal();

    reference_path_ = goal->path;
    following_mode_ = goal->following_mode;

    ph_.setPath(reference_path_);

    if(goal->clear_trajectory)
    {
      robot_trajectory_.poses.clear();
      cart_trajectory_.poses.clear();
    }

    cmd_vel_timer_.start();
    busy_.data = true; // 動作状態を動作中に
    ROS_INFO("Received new reference path");
  }

  // PathFollowingアクションの中断コールバック
  void preemptCB()
  {
    if(action_server_.isNewGoalAvailable())
    {
      goalCB();
    }
    else
    {
      stopFollowing();
      action_server_.setPreempted();
      busy_.data = false; // 動作状態をスタンバイに
      ROS_INFO("Path following canceled");
    }
  }

  void reconfigureCB(agv_navigation::PathFollowingControllerConfig& config,
                     uint32_t level)
  {
    v_forward_ = config.vel_x_forward;
    v_backward_ = config.vel_x_backward;
    omega_max_ = config.vel_theta_max;

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
    dt_ = config.control_period;
    t_p_ = config.prediction_horizon;

    cmd_vel_timer_.setPeriod(ros::Duration(dt_));
  }

  ros::NodeHandle nh_;

  ros::Publisher reference_path_pub_, predicted_path_pub_,
                 robot_trajectory_pub_, cart_trajectory_pub_,
                 pose_array_pub_, cmd_vel_pub_, busy_pub_;
  ros::Subscriber cart_yaw_sub_;

  ros::Timer cmd_vel_timer_, status_timer_;

  ros::ServiceServer show_trajectory_service_server_;

  PathFollowingActionServer action_server_;

  PathFollowingControllerConfigServer config_server_;

  RvizHandler rh_;

  TFHandler th_;
  std::string global_frame_, robot_base_frame_, coupler_frame_, cart_base_frame_;
  std::array<geometry_msgs::PoseStamped, NUM_OF_TARGETS> target_pose_;

  PathHandler ph_;
  nav_msgs::Path reference_path_, robot_trajectory_, cart_trajectory_;

  std_msgs::Bool busy_;

  bool is_towing_;
  float phi_;
  double l_1_, l_2_;

  double v_forward_, v_backward_, omega_max_;

  int following_mode_;

  std::array<std::array<double, NUM_OF_TARGETS>, 2> q_e_, q_theta_;
  double r_, r_d_;
  double dt_, t_p_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_following_controller");

  PathFollowingController path_following_controller;

  ros::spin();

  return 0;
}