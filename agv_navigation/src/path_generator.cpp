#include <Eigen/Dense>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <tf2/utils.h>

#include <agv_control_lib/path_handler.h>
#include <agv_control_lib/tf_handler.h>
#include <agv_control_msgs/DeletePath.h>
#include <agv_control_msgs/GeneratePath.h>
#include <agv_control_msgs/ShowEntirePath.h>
#include <agv_control_msgs/OperatePathFollowing.h>
#include <agv_control_msgs/PathGeneratorConfig.h>
#include <agv_control_msgs/PathFollowingAction.h>

class PathGenerator
{
public:

  using PathFollowingActionClient =
    actionlib::SimpleActionClient<agv_control_msgs::PathFollowingAction>;
  using PathGeneratorConfigServer =
    dynamic_reconfigure::Server<agv_navigation::PathGeneratorConfig>;

  PathGenerator() :
  pnh_("~"),
  path_following_action_client_("path_following"),
  is_path_following_in_progress_(false),
  is_path_following_finished_(false)
  {
    ros::NodeHandle nh;
    entire_path_pub_ = pnh_.advertise<nav_msgs::Path>("entire_path", 1);
    timer_ = nh.createTimer(ros::Duration(0.1), &PathGenerator::timerCB,
                            this, false, false);

    pnh_.param("path_resolution", path_resolution_, 0.05);
    pnh_.param("arc_curvature", arc_curvature_, 0.5);
    pnh_.param("arc_angle", arc_angle_, 3.14);
    pnh_.param("line_length", line_length_, 1.0);
    pnh_.param("line_angle", line_angle_, 0.0);

    int _path_direction;
    pnh_.param("path_direction", _path_direction, (int)Direction::FORWARD);
    path_direction_ = (Direction)_path_direction;

    pnh_.param("global_frame", global_frame_, std::string(""));
    pnh_.param("robot_base_frame", robot_base_frame_, std::string(""));

    delete_path_service_server_ =
      pnh_.advertiseService("delete_path",
                          &PathGenerator::deletePath, this);
    generate_path_service_server_ =
      pnh_.advertiseService("generate_path",
                          &PathGenerator::generatePath, this);
    show_entire_path_service_server_ =
      pnh_.advertiseService("show_entire_path",
                          &PathGenerator::showEntirePath, this);
    operate_path_following_service_server_ =
      pnh_.advertiseService("operate_path_following",
                          &PathGenerator::operatePathFollowing, this);

    config_server_.setCallback(boost::bind(&PathGenerator::reconfigureCB, this, _1, _2));

    if(!path_following_action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("Path following action server timeout");
    }

    th_.setGlobalFrame(global_frame_);

    entire_path_.header.frame_id = global_frame_;
  }

private:

  enum Direction
  {
    FORWARD,
    BACKWARD
  };

  // 経路を生成(appendがtrueで直前の経路に接続)
  bool generatePath(bool append)
  {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = global_frame_;

    bool backward = (path_direction_ == Direction::BACKWARD);

    geometry_msgs::PoseStamped start_pose;
    double start_yaw;

    if(!append && !path_list_.empty()) path_list_.pop_back();

    if(path_list_.empty()) // 経路がない場合はAGVの現在位置を始点に
    {
      if(!th_.getGlobalPose(robot_base_frame_, start_pose))
      {
        ROS_ERROR("Could not get robot pose");
        return false;
      }
      start_yaw = tf2::getYaw(start_pose.pose.orientation);
    }
    else // すでに経路がある場合は直前の経路の終点を始点に
    {
      ph_.setPath(path_list_.back().second);
      auto start_pose_itr = ph_.getEnd();
      start_pose = *start_pose_itr;
      start_yaw = ph_.getTangentYaw(start_pose_itr);
      if(path_list_.back().first == Direction::BACKWARD) start_yaw += M_PI;
    }

    Eigen::Vector2d start_position_vec(start_pose.pose.position.x,
                                       start_pose.pose.position.y);
    Eigen::Vector2d start_direction_vec(std::cos(start_yaw),
                                        std::sin(start_yaw));
    if(backward) start_direction_vec *= -1;

    if(arc_curvature_ == 0) // 直線の場合
    {
      Eigen::Vector2d line_direction_vec =
        Eigen::Rotation2D(line_angle_)*start_direction_vec;
      for(double distance = 0; distance < line_length_; distance += path_resolution_)
      {
        path.poses.push_back(
          vecToPose(start_position_vec + distance*line_direction_vec));
      }
      path.poses.push_back(
        vecToPose(start_position_vec + line_length_*line_direction_vec));
    }
    else // 曲線の場合は始点で接する円弧を作る(半径方向のベクトルを回転させて打点)
    {
      double arc_radius = 1/arc_curvature_; // can be negative
      Eigen::Vector2d arc_radius_vec =
        -arc_radius*start_direction_vec.unitOrthogonal();
      Eigen::Vector2d arc_center_vec = start_position_vec - arc_radius_vec;

      for(double theta = 0; std::abs(theta) <= arc_angle_;
          theta += path_resolution_/arc_radius)
      {
        path.poses.push_back(
          vecToPose(arc_center_vec + Eigen::Rotation2D(theta)*arc_radius_vec));
      }
      path.poses.push_back(
          vecToPose(arc_center_vec + Eigen::Rotation2D(std::copysign(arc_angle_, arc_radius))
                                     *arc_radius_vec));
    }

    // 経路リスト(path_list_)に作成した経路を追加
    path_list_.emplace_back(path_direction_, path);

    // 全経路を更新
    refreshEntirePath();

    return true;
  }

  // 全経路を更新
  void refreshEntirePath()
  {
    entire_path_.poses.clear();
    entire_path_.header.stamp = ros::Time::now();

    // 経路リストに入っているすべての経路(を構成する点)を一つの経路にまとめる
    for(auto itr = path_list_.begin(); itr != path_list_.end(); ++itr)
    {
      auto& path_poses = (*itr).second.poses;
      std::copy(path_poses.begin(), path_poses.end(),
                std::back_inserter(entire_path_.poses));
    }

    // 全経路をpub
    entire_path_pub_.publish(entire_path_);
  }

  // 経路を削除
  bool deletePath(agv_control_msgs::DeletePathRequest& req,
                  agv_control_msgs::DeletePathResponse& res)
  {
    if(path_list_.empty()) return false;

    if(req.all) // allがtrueの場合は経路リストに入っているすべての経路を削除
    {
      path_list_.clear();
    }
    else // allがfalseの場合は直前の経路のみ削除
    {
      path_list_.pop_back();
    }

    // 全経路を更新
    refreshEntirePath();

    return true;
  }
  
  // 経路を生成(サービスのコールバック) 
  bool generatePath(agv_control_msgs::GeneratePathRequest& req,
                    agv_control_msgs::GeneratePathResponse& res)
  {
    // 経路追従が終了してから初めて経路を作成する場合は、前の経路が残っているので削除
    if(is_path_following_finished_)
    {
      is_path_following_finished_ = false;
      path_list_.clear();
    }
    
    // プリセットの経路を作成する場合のみ各パラメータをリクエストから設定
    // (Generate Pathボタンで作成する場合はリクエストのパラメータがすべて0になっており、Dynamic Reconfigureで設定された値を使用)
    if(req.path_resolution != 0.0)
    {
      path_resolution_ = req.path_resolution;
      arc_curvature_ = req.arc_curvature;
      arc_angle_ = req.arc_angle;
      line_length_ = req.line_length;
      line_angle_ = req.line_angle;
      path_direction_ = (Direction)req.path_direction;
    }

    generatePath(true);
    
    res.path = entire_path_;

    return true;
  }

  // 全経路を表示
  bool showEntirePath(agv_control_msgs::ShowEntirePathRequest& req,
                      agv_control_msgs::ShowEntirePathResponse& res)
  {
    refreshEntirePath();

    res.entire_path = entire_path_;

    return true;
  }

  // 経路追従を開始・停止
  bool operatePathFollowing(agv_control_msgs::OperatePathFollowingRequest& req,
                            agv_control_msgs::OperatePathFollowingResponse& res)
  {
    if(req.stop) // 経路追従停止の場合
    {
      timer_.stop();
      path_following_action_client_.cancelAllGoals();
      is_path_following_in_progress_ = false; // 経路追従進行状態を停止に
      is_path_following_finished_ = true;     // 経路追従完了
    }
    else
    {
      if(!is_path_following_in_progress_ && !path_list_.empty())
      {
        reference_path_itr_ = path_list_.cbegin(); // 経路リストの先頭を参照経路に設定
        sendReferencePath(true); // 参照経路をpub

        timer_.start();
        is_path_following_in_progress_ = true; // 経路追従進行状態を進行中に
        is_path_following_finished_ = false;
      }
    }

    return true;
  }

  // Dynamic Reconfigureのコールバック
  void reconfigureCB(const agv_navigation::PathGeneratorConfig& config, uint32_t level)
  {
    path_resolution_ = config.path_resolution;
    arc_curvature_ = config.arc_curvature;
    arc_angle_ = config.arc_angle;
    line_length_ = config.line_length;
    line_angle_ = config.line_angle;
    path_direction_ = (Direction)config.path_direction;

    if(!path_list_.empty() && !is_path_following_finished_) generatePath(false);
  }

  // タイマーのコールバック
  void timerCB(const ros::TimerEvent& e)
  {
    if(path_following_action_client_.getState() ==
       actionlib::SimpleClientGoalState::SUCCEEDED) // 経路追従が完了している場合
    {
      if(++reference_path_itr_ == path_list_.cend()) // 参照経路が経路リストの最後だった場合(そうでなければ次に進める)
      {
        timer_.stop();
        is_path_following_in_progress_ = false; // 経路追従進行状態を停止に
        is_path_following_finished_ = true;     // 経路追従完了
      }
      else
      {
        sendReferencePath(false);
      }
    }
    else if(path_following_action_client_.getState() !=
            actionlib::SimpleClientGoalState::ACTIVE) // 経路追従が進行中でない(中断など)場合
    {
      timer_.stop();
      is_path_following_in_progress_ = false; // 経路追従進行状態を停止に
      is_path_following_finished_ = true;     // 経路追従完了
    }
  }

  // Eigenのベクトルをgeometry_msgs/PoseStampedに変換
  geometry_msgs::PoseStamped vecToPose(const Eigen::Vector2d path_point_vec)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = path_point_vec.x();
    pose.pose.position.y = path_point_vec.y();

    return pose;
  }
  
  // 参照経路をpub
  void sendReferencePath(bool clear_trajectory)
  { 
    uint8_t following_mode = (*reference_path_itr_).first;
    nav_msgs::Path path = (*reference_path_itr_).second;
    
    // 参照経路の方向が次の経路と同じ場合は一本につなげる
    while(std::next(reference_path_itr_) != path_list_.cend())
    {
      if((*std::next(reference_path_itr_)).first != following_mode) break;
      ROS_INFO("Path joined");
      auto& poses = (*++reference_path_itr_).second.poses;
      std::copy(poses.begin(), poses.end(), std::back_inserter(path.poses));
    }

    // PathFollowingアクションのゴールとしてpub
    agv_control_msgs::PathFollowingGoal action_goal;
    action_goal.following_mode = following_mode;
    action_goal.path = path;
    action_goal.clear_trajectory = clear_trajectory;
    path_following_action_client_.sendGoal(action_goal);
  }

  ros::NodeHandle pnh_;

  ros::Publisher entire_path_pub_;

  ros::Timer timer_;

  ros::ServiceServer delete_path_service_server_,
                     generate_path_service_server_,
                     show_entire_path_service_server_,
                     operate_path_following_service_server_;

  PathGeneratorConfigServer config_server_;

  PathFollowingActionClient path_following_action_client_;

  PathHandler ph_;

  TFHandler th_;
  std::string global_frame_, robot_base_frame_;

  std::vector<std::pair<Direction, nav_msgs::Path>> path_list_;
  std::vector<std::pair<Direction, nav_msgs::Path>>::const_iterator reference_path_itr_;
  nav_msgs::Path entire_path_;

  bool is_path_following_in_progress_, is_path_following_finished_;

  double path_resolution_, arc_curvature_, arc_angle_, line_length_, line_angle_;
  Direction path_direction_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_generator");
  
  PathGenerator path_generator;
  
  ros::spin();
  
  return 0;
}