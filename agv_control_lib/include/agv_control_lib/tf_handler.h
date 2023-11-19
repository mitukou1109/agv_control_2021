#ifndef AGV_CONTROL_LIB_TF_HANDLER_H_
#define AGV_CONTROL_LIB_TF_HANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

class TFHandler
{
public:

  TFHandler(const std::string& global_frame = "");

  bool getGlobalPose(const std::string& base_frame,
                     geometry_msgs::PoseStamped& global_pose,
                     ros::Duration timeout = ros::Duration(0));

  bool getOriginDistance(const std::string& base_frame_1,
                         const std::string& base_frame_2,
                         double& distance,
                         ros::Duration timeout = ros::Duration(0));
  
  void setGlobalFrame(const std::string& global_frame)
  { global_frame_ = global_frame; };

private:

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string global_frame_;
};

#endif // AGV_CONTROL_LIB_TF_HANDLER_H_