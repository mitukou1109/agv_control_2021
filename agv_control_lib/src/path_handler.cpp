#include "agv_control_lib/path_handler.h"

#include <algorithm>
#include <random>
#include <tf2/utils.h>

PathHandler::PathHandler() {}

// 経路(path_)を構成する点のうち現在位置(target_pose)に最も近いものを取得
PathHandler::_path_itr_type PathHandler::getReferencePoint(const _pose_type& target_pose)
{
  // 位置a、bのどちらが現在位置に近いかを返すファンクタ
  auto comparator =
    [&target_pose](const _pose_type& a, const _pose_type& b)
    {
      double a_distance = std::hypot(a.pose.position.x-target_pose.pose.position.x,
                                     a.pose.position.y-target_pose.pose.position.y);
      double b_distance = std::hypot(b.pose.position.x-target_pose.pose.position.x,
                                     b.pose.position.y-target_pose.pose.position.y);
      return (a_distance < b_distance);
    };
  
  // 配列(path_.poses)の中でcomparatorが定める最小の要素を返す
  return std::min_element(path_.poses.begin(), path_.poses.end(), comparator);
}

// 参照点(ref_point)における経路の曲率を計算
double PathHandler::getCurvature(const _path_itr_type ref_point, double sample_length)
{
  // 参照点の近傍で一定の長さ(sample_length)の経路を取り出す
  // 中間点(middle_point)からsample_length/2だけ前後の点を取る
  auto middle_point = ref_point;

  // 参照点の前にsample_length/2の長さがない場合、
  // 経路の始点からsample_length/2先の点を中間点に設定
  if(getSegmentLength(getStart(), middle_point) <= sample_length/2)
  {
    middle_point = getSegmentEnd(getStart(), sample_length/2, false);
  }
  // 参照点の後にsample_length/2の長さがない場合、
  // 経路の終点からsample_length/2前の点を中間点に設定
  else if(getSegmentLength(middle_point, getEnd()) <= sample_length/2)
  {
    middle_point = getSegmentEnd(getEnd(), sample_length/2, true);
  }

  // 中間点からsample_length/2先の点を取得
  auto forward_point = getSegmentEnd(middle_point, sample_length/2, false);
  // 中間点からsample_length/2前の点を取得
  auto backward_point = getSegmentEnd(middle_point, sample_length/2, true);
  
  // 中間点と前後の点を結ぶ線分の傾きの差(yaw_variance)を計算
  double yaw_variance =
    getSegmentYaw(ref_point, forward_point) - getSegmentYaw(backward_point, ref_point);
  if(std::abs(yaw_variance) >= M_PI)
    yaw_variance -= std::copysign(2*M_PI, yaw_variance);

  // 接線の傾きの差をsample_lengthで割って曲率を計算
  return yaw_variance / sample_length;
}

// 参照点(ref_point)における経路の接線の傾きを計算
double PathHandler::getTangentYaw(_path_itr_type ref_point)
{
  // 参照点が経路の終点の場合は、一つ前の点を参照点に設定
  if(ref_point == getEnd()) ref_point = std::prev(getEnd());
  // 参照点と一つ先の点を結ぶ線分の傾きを計算
  return getSegmentYaw(ref_point, getNextPoint(ref_point));
}

// 経路全体の長さを取得
double PathHandler::getEntireLength()
{
  return getSegmentLength(getStart(), getEnd());
}

// 経路上の点(point)の一つ先を取得
PathHandler::_path_itr_type PathHandler::getNextPoint(const _path_itr_type point)
{
  return (point != getEnd() ? std::next(point) : point);
}

// 渡された経路(path)全体の長さを取得
double PathHandler::getPathLength(const nav_msgs::Path& path)
{
  return getSegmentLength(path.poses.begin(), path.poses.end());
}

// 始点(start)からsegment_lengthだけ先or前の点を取得
PathHandler::_path_itr_type PathHandler::getSegmentEnd(
  const _path_itr_type start,
  double segment_length,
  bool reverse)
{
  if((start == getEnd() && !reverse) ||
     (start == getStart() && reverse))  return start;

  // reverseがtrueの場合は先、falseの場合は前を見る
  int increment = reverse ? -1 : 1;
  auto limit = reverse ? getStart() : getEnd();

  auto itr = std::next(start, increment);
  while(itr != limit)
  {
    if(getSegmentLength(start, itr) >= segment_length) break;
    std::advance(itr, increment);
  }

  return itr;
}

// 始点(start)から終点(end)までの経路の長さを取得
double PathHandler::getSegmentLength(const _path_itr_type start,
                                     const _path_itr_type end)
{
  int increment = (std::distance(start, end) > 0) ? 1 : -1;
  double length = 0;
  for(auto itr = start; itr != end; std::advance(itr, increment))
  {
    auto& position = (*itr).pose.position;
    auto& next_position = (*std::next(itr, increment)).pose.position;
    length += std::hypot(position.x-next_position.x, position.y-next_position.y);
  }
  return length;
}

// 始点(start)と終点(end)を結ぶ線分の傾きを計算
double PathHandler::getSegmentYaw(const _path_itr_type start, const _path_itr_type end)
{
  return std::atan2((*end).pose.position.y-(*start).pose.position.y,
                    (*end).pose.position.x-(*start).pose.position.x);
}

// 経路の近似円を求める
PathHandler::Circle PathHandler::circleFit(const _path_type& path, unsigned int num_of_samples)
{
  // 経路を構成する点の中からランダムにnum_of_samplesの数だけ取り出す
  auto samples = _path_type::_poses_type(num_of_samples);
  std::sample(path.poses.begin(), path.poses.end(), samples.begin(), samples.size(),
              std::mt19937((std::random_device())()));

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);

  // 円フィッティング(参考：第17回定例報告スライド)
  for (const auto& pose : samples)
  {
    const auto& point = pose.pose.position;
    std::array<double, 3> c = {point.x, point.y, 1};
    for (int m = 0; m < 3; m++)
    {
      for (int n = 0; n < 3; n++)
      {
        A(m, n) += c.at(m) * c.at(n);
      }
      b(m) -= c.at(m) * (std::pow(point.x, 2)+std::pow(point.y, 2));
    }
  }
  Eigen::VectorXd coef = A.ldlt().solve(b);

  Circle circle;
  circle.center.header.frame_id = path.header.frame_id;
  circle.center.header.stamp = ros::Time::now();
  circle.center.point.x = -coef(0)/2;
  circle.center.point.y = -coef(1)/2;
  circle.radius = std::sqrt((std::pow(coef(0), 2)+std::pow(coef(1), 2))/4 - coef(2));

  return circle;
}

// 経路の近似n次曲線を求める
std::vector<double> PathHandler::curveFit(const _path_type& path)
{
  constexpr int DEG = 4; // 近似曲線の次数
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(DEG+1, DEG+1);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(DEG+1);
  
  // カーブフィッティング(参考：第15回定例報告スライド)
  for (const auto& pose : path.poses)
  {
    auto point = pose.pose.position;
    for (int m = 0; m < DEG+1; m++)
    {
      for (int n = 0; n < DEG+1; n++)
      {
        A(m, n) += std::pow(point.x, DEG*2-(m+n));
      }
      b(m) += std::pow(point.x, DEG-m) * point.y;
    }
  }
  Eigen::VectorXd coef = A.ldlt().solve(b);

  std::vector<double> coef_vec(coef.rows());
  for (int i = 0; i < coef.rows(); i++)
  {
    coef_vec.push_back(coef(i));
  }

  return coef_vec;
    // [coef](double x)
    // {
    //   double y = 0;
    //   for (int i = 0; i < coef.rows(); i++)
    //   {
    //     y += coef(i)*std::pow(x, DEG-i);
    //   }
    //   geometry_msgs::Point point;
    //   point.x = x;
    //   point.y = y;
    //   return point;
    // };
}