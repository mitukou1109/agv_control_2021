#ifndef AGV_CONTROL_LIB_PATH_HANDLER_H_
#define AGV_CONTROL_LIB_PATH_HANDLER_H_

#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

class PathHandler 
{
public:

  using _path_type = nav_msgs::Path;
  using _path_itr_type = nav_msgs::Path::_poses_type::const_iterator;
  using _pose_type = geometry_msgs::PoseStamped;

  struct Circle
  {
    geometry_msgs::PointStamped center;
    double radius;
  };

  PathHandler();

  _path_itr_type getReferencePoint(const _pose_type& target_pose);
  
  double getCurvature(const _path_itr_type ref_point, double sample_length);

  double getTangentYaw(const _path_itr_type ref_point);

  double getEntireLength();

  _path_itr_type getSegmentEnd(const _path_itr_type start,
                               double segment_length,
                               bool reverse = false);

  _path_itr_type getNextPoint(const _path_itr_type point);

  _path_itr_type getStart() { return path_.poses.cbegin(); }

  _path_itr_type getEnd() { return std::prev(path_.poses.cend()); }

  void setPath(const _path_type& path) { path_ = path; };

  static double getPathLength(const nav_msgs::Path& path);

  static double getSegmentLength(const _path_itr_type start,
                                 const _path_itr_type end);
  
  static double getSegmentYaw(const _path_itr_type start, const _path_itr_type end);

private:

  Circle circleFit(const _path_type& path, unsigned int num_of_samples);

  std::vector<double> curveFit(const _path_type& path);

  _path_type path_;
};

#endif // AGV_CONTROL_LIB_PATH_HANDLER_H_