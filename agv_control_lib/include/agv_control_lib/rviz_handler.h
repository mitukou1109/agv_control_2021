#ifndef AGV_CONTROL_LIB_RVIZ_HANDLER_H_
#define AGV_CONTROL_LIB_RVIZ_HANDLER_H_

#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <visualization_msgs/Marker.h>

class RvizHandler
{
public:
  RvizHandler();
  
  void drawLine(
    const std::string& origin_frame,
    const std::string& ns,
    const visualization_msgs::Marker::_points_type& points,
    double width,
    std::array<float, 3> rgb,
    double alpha = 1.0);
  
  void drawLine(
    const std::string& origin_frame,
    const std::string& ns,
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& end,
    double width,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void drawLine(
    const std::string& origin_frame,
    const std::string& ns,
    std::array<double, 3> start,
    std::array<double, 3> end, 
    double width,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void drawLineFromOrigin(
    const std::string& origin_frame,
    const std::string& ns,
    const geometry_msgs::Point& end,
    double width,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void drawLineFromOrigin(
    const std::string& origin_frame,
    const std::string& ns,
    std::array<double, 3> end,
    double width,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void drawCircle(
    const std::string& origin_frame,
    const std::string& ns,
    const geometry_msgs::Point& center,
    double radius,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void drawCircle(
    const std::string& origin_frame,
    const std::string& ns,
    std::array<double, 2> center,
    double radius,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void showText(
    const std::string& origin_frame,
    const std::string& ns,
    const geometry_msgs::Point& position,
    const std::string& text,
    std::array<float, 3> rgb,
    double alpha = 1.0);

  void showOverlayText(const std::string& text);

  void clearLine(const std::string& ns);

  void clearCircle(const std::string& ns);

  void clearText(const std::string& ns);

private:
  void clear(const std::string& ns);

  ros::Publisher marker_pub_, text_pub_;

  int line_id_, circle_id_, text_id_;
};

#endif // AGV_CONTROL_LIB_RVIZ_HANDLER_H_