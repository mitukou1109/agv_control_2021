#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>

#include <agv_control_msgs/DeletePath.h>
#include <agv_control_msgs/GeneratePath.h>
#include <agv_control_msgs/ShowEntirePath.h>
#include <agv_control_msgs/ShowTrajectory.h>
#include <agv_control_msgs/OperatePathFollowing.h>
#endif

#include <QPushButton>

namespace agv_navigation
{

const std::vector<std::vector<std::array<double, 6>>> PRESET({
  {
    {0.05, 0.0, 0.0, 1.75, 0.0, 0},
    {0.05, 0.7, 1.57, 0.0, 0.0, 0},
    {0.05, 0.0, 0.0, 2.0, 0.0, 0}
  },
  {
    {0.05, 0.0, 0.0, 1.75, 0.0, 1},
    {0.05, 0.7, 1.57, 0.0, 0.0, 1},
    {0.05, 0.0, 0.0, 1.0, 0.0, 1}
  }
});

class NavigationControlPanel : public rviz::Panel
{
  Q_OBJECT
public:

  using DeletePathService = agv_control_msgs::DeletePath;
  using GeneratePathService = agv_control_msgs::GeneratePath;
  using ShowEntirePathService = agv_control_msgs::ShowEntirePath;
  using ShowTrajectoryService = agv_control_msgs::ShowTrajectory;
  using OperatePathFollowingService = agv_control_msgs::OperatePathFollowing;

  NavigationControlPanel(QWidget* parent = 0);

private:

  struct PathGenerationButtons
  {
    QPushButton* generate_path_button;
    std::vector<QPushButton*> preset_buttons;
    QPushButton* show_path_button;
    QPushButton* delete_last_path_button;
    QPushButton* delete_all_paths_button;
  };

  struct PathFollowingButtons
  {
    QPushButton* start_following_button;
    QPushButton* stop_following_button;
  };

  void deletePath(bool all);

  void generatePath();

  void generatePath(const std::array<double, 6>& param);

  void generatePresetPath(int id);

  void showPath();

  void operatePathFollowing(bool stop);

  PathGenerationButtons path_generation_buttons_;

  PathFollowingButtons path_following_buttons_;

  ros::ServiceClient generate_path_service_client_,
                     delete_path_service_client_,
                     show_entire_path_service_client_,
                     show_trajectory_service_client_,
                     operate_path_following_service_client_;

  std::string navigation_service_prefix_;
};

} // namespace agv_navigation