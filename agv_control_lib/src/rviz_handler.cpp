#include "agv_control_lib/rviz_handler.h"
#include <std_msgs/Float32.h>

RvizHandler::RvizHandler()
{
  ros::NodeHandle nh;
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("marker", 1);
  text_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1);
}

// Rvizの画面上に線を描画
void RvizHandler::drawLine(
  const std::string& origin_frame, // pointsの座標系
  const std::string& ns,           // pubする線の名前
  const visualization_msgs::Marker::_points_type& points, // 線を構成する点
  double width,             // 線の幅
  std::array<float, 3> rgb, // 線の色
  double alpha)             // 線の透明度(0〜1)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = origin_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = line_id_++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = rgb.at(0);
  marker.color.g = rgb.at(1);
  marker.color.b = rgb.at(2);
  marker.color.a = alpha;
  marker.points = points;

  marker_pub_.publish(marker);
}

// drawLine()の別バージョン
void RvizHandler::drawLine(
  const std::string& origin_frame,
  const std::string& ns,
  const geometry_msgs::Point& start, // 直線の始点
  const geometry_msgs::Point& end,   // 直線の終点
  double width,
  std::array<float, 3> rgb,
  double alpha)
{
  drawLine(origin_frame, ns, {start, end}, width, rgb, alpha);
}

// drawLine()の別バージョン
void RvizHandler::drawLine(
  const std::string& origin_frame,
  const std::string& ns,
  std::array<double, 3> start,
  std::array<double, 3> end,
  double width,
  std::array<float, 3> rgb,
  double alpha)
{
  geometry_msgs::Point _start, _end;
  _start.x = start.at(0);
  _start.y = start.at(1);
  _start.z = start.at(2);
  _end.x = end.at(0);
  _end.y = end.at(1);
  _end.z = end.at(2);
  drawLine(origin_frame, ns, _start, _end, width, rgb, alpha);
}

// drawLine()の始点が座標系原点のバージョン
void RvizHandler::drawLineFromOrigin(
  const std::string& origin_frame,
  const std::string& ns,
  const geometry_msgs::Point& end,
  double width,
  std::array<float, 3> rgb,
  double alpha)
{
  geometry_msgs::Point origin;
  origin.x = origin.y = origin.z = 0;
  drawLine(origin_frame, ns, {origin, end}, width, rgb, alpha);
}

// drawLineFromOrigin()の別バージョン
void RvizHandler::drawLineFromOrigin(
  const std::string& origin_frame,
  const std::string& ns,
  std::array<double, 3> end,
  double width,
  std::array<float, 3> rgb,
  double alpha)
{
  drawLine(origin_frame, ns, {0, 0, 0}, end, width, rgb, alpha);
}

// Rvizの画面上に円(高さ0.001mの円柱)を描画
void RvizHandler::drawCircle(
    const std::string& origin_frame,
    const std::string& ns,
    const geometry_msgs::Point& center, // 円の中心
    double radius,                      // 円の半径
    std::array<float, 3> rgb,
    double alpha)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = origin_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = circle_id_++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = radius*2;
  marker.scale.z = 0.001;
  marker.pose.position = center;
  marker.color.r = rgb.at(0);
  marker.color.g = rgb.at(1);
  marker.color.b = rgb.at(2);
  marker.color.a = alpha;

  marker_pub_.publish(marker);
}

// drawCircle()の別バージョン
void RvizHandler::drawCircle(
    const std::string& origin_frame,
    const std::string& ns,
    std::array<double, 2> center,
    double radius,
    std::array<float, 3> rgb,
    double alpha)
{
  geometry_msgs::Point _center;
  _center.x = center.at(0);
  _center.y = center.at(1);

  drawCircle(origin_frame, ns, _center, radius, rgb, alpha);
}

// Rvizの画面上に文字列を表示(空間上で立体的に表示)
void RvizHandler::showText(
  const std::string& origin_frame,
  const std::string& ns,
  const geometry_msgs::Point& position, // 
  const std::string& text,
  std::array<float, 3> rgb,
  double alpha)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = origin_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = text_id_++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.pose.position = position;
  marker.color.r = rgb.at(0);
  marker.color.g = rgb.at(1);
  marker.color.b = rgb.at(2);
  marker.color.a = alpha;
  marker.text = text;

  marker_pub_.publish(marker);
}

// Rvizの画面上に文字を表示(Displayにオーバーレイ表示)
void RvizHandler::showOverlayText(const std::string& text)
{
  jsk_rviz_plugins::OverlayText _text;

  _text.action = jsk_rviz_plugins::OverlayText::ADD;
  _text.width = 400;
  _text.height = 150;
  _text.left = 20;
  _text.top = 700;

  _text.bg_color.r = 0;
  _text.bg_color.g = 0;
  _text.bg_color.b = 0;
  _text.bg_color.a = 0.4;

  _text.fg_color.r = 0.1;
  _text.fg_color.g = 1.0;
  _text.fg_color.b = 1.0;
  _text.fg_color.a = 0.8;

  _text.line_width = 1;
  _text.text_size = 32;
  _text.font = "Ubuntu";
  _text.text = text;
  
  text_pub_.publish(_text);
}

// 名前がnsの線を消去
void RvizHandler::clearLine(const std::string& ns)
{
  line_id_ = 0;
  clear(ns);
}

// 名前がnsの円を消去
void RvizHandler::clearCircle(const std::string& ns)
{
  circle_id_ = 0;
  clear(ns);
}

// 名前がnsの文字列を消去
void RvizHandler::clearText(const std::string& ns)
{
  text_id_ = 0;
  clear(ns);
}

// 名前がnsの表示をすべて消去
void RvizHandler::clear(const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::DELETEALL;

  marker_pub_.publish(marker);
}