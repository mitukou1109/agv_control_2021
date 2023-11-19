#include "agv_control_lib/tf_handler.h"

TFHandler::TFHandler(const std::string& global_frame) :
tf_listener_(tf_buffer_)
{
  global_frame_ = global_frame;
}

// 渡された座標系(base_frame)の原点の、基準座標系(global_frame_)における位置を計算
// (global_frame→base_frameのTFを取得)
bool TFHandler::getGlobalPose(const std::string& base_frame,
                              geometry_msgs::PoseStamped& global_pose, // 位置の代入先
                              ros::Duration timeout)
{
  try
  {
    auto tf = tf_buffer_.lookupTransform(global_frame_, base_frame, ros::Time(0), timeout);
    global_pose.header.frame_id = global_frame_;
    global_pose.header.stamp = tf.header.stamp;
    global_pose.pose.orientation = tf.transform.rotation;
    global_pose.pose.position.x = tf.transform.translation.x;
    global_pose.pose.position.y = tf.transform.translation.y;
    global_pose.pose.position.z = tf.transform.translation.z;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  return true;
}

// 2つの座標系の原点間距離を取得
bool TFHandler::getOriginDistance(const std::string& base_frame_1,
                                  const std::string& base_frame_2,
                                  double& distance,
                                  ros::Duration timeout)
{
  try
  {
    auto tf = tf_buffer_.lookupTransform(base_frame_1, base_frame_2, ros::Time(0), timeout);
    distance = std::hypot(tf.transform.translation.x, tf.transform.translation.y);
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  return true;
}