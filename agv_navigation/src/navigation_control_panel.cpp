#include "agv_navigation/navigation_control_panel.h"

#include <pluginlib/class_list_macros.h>
#include <QGridLayout>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QTimer>

#include <geometry_msgs/Twist.h>

namespace agv_navigation
{

NavigationControlPanel::NavigationControlPanel(QWidget* parent) :
  rviz::Panel(parent)
{
  ros::NodeHandle pnh("~");

  pnh.param("navigation_service_prefix", navigation_service_prefix_, std::string(""));

  ros::NodeHandle
    path_generator_nh(navigation_service_prefix_+"/path_generator"),
    path_following_controller_nh(navigation_service_prefix_+"/path_following_controller");

  generate_path_service_client_ =
    path_generator_nh.serviceClient<GeneratePathService>("generate_path");
  delete_path_service_client_ =
    path_generator_nh.serviceClient<DeletePathService>("delete_path");
  show_entire_path_service_client_ =
    path_generator_nh.serviceClient<ShowEntirePathService>("show_entire_path");
  operate_path_following_service_client_ =
    path_generator_nh.serviceClient<OperatePathFollowingService>("operate_path_following");
  show_trajectory_service_client_ =
    path_following_controller_nh.serviceClient<ShowTrajectoryService>("show_trajectory");
    
  path_generation_buttons_.generate_path_button = new QPushButton("Generate new path");
  path_generation_buttons_.show_path_button = new QPushButton("Show path");
  path_generation_buttons_.delete_last_path_button = new QPushButton("Delete last path");
  path_generation_buttons_.delete_all_paths_button = new QPushButton("Delete all paths");
  path_following_buttons_.start_following_button = new QPushButton("Start following");
  path_following_buttons_.stop_following_button = new QPushButton("Stop following");

  auto& preset_path_buttons = path_generation_buttons_.preset_buttons;
  preset_path_buttons.resize(PRESET.size());
  for(int i = 0; i < PRESET.size(); i++)
    preset_path_buttons.at(i) = new QPushButton(("Preset "+std::to_string(i+1)).c_str());

  QVBoxLayout* layout_base = new QVBoxLayout;

  QGroupBox* group_path_generation = new QGroupBox("Path generation");
  QVBoxLayout* layout_path_generation = new QVBoxLayout;
  QGridLayout* grid_presets = new QGridLayout;
  layout_path_generation->addWidget(path_generation_buttons_.generate_path_button);
  for(int i = 0; i < PRESET.size(); i++)
    grid_presets->addWidget(preset_path_buttons.at(i), i/2, i%2);
  layout_path_generation->addLayout(grid_presets);
  layout_path_generation->addWidget(path_generation_buttons_.show_path_button);
  layout_path_generation->addWidget(path_generation_buttons_.delete_last_path_button);
  layout_path_generation->addWidget(path_generation_buttons_.delete_all_paths_button);
  group_path_generation->setLayout(layout_path_generation);

  QGroupBox* group_path_following = new QGroupBox("Path following");
  QVBoxLayout* layout_path_following = new QVBoxLayout;
  layout_path_following->addWidget(path_following_buttons_.start_following_button);
  layout_path_following->addWidget(path_following_buttons_.stop_following_button);
  group_path_following->setLayout(layout_path_following);

  layout_base->addWidget(group_path_generation);
  layout_base->addWidget(group_path_following);
  setLayout(layout_base);

  connect(path_generation_buttons_.generate_path_button, &QPushButton::clicked, this,
          (void(NavigationControlPanel::*)())&NavigationControlPanel::generatePath);
  for(int i = 0; i < PRESET.size(); i++)
  {
    connect(preset_path_buttons.at(i), &QPushButton::clicked, this,
            std::bind(&NavigationControlPanel::generatePresetPath, this, i));
  }
  connect(path_generation_buttons_.show_path_button, &QPushButton::clicked, this,
          &NavigationControlPanel::showPath);
  connect(path_generation_buttons_.delete_last_path_button, &QPushButton::clicked, this,
          std::bind(&NavigationControlPanel::deletePath, this, false));
  connect(path_generation_buttons_.delete_all_paths_button, &QPushButton::clicked, this,
          std::bind(&NavigationControlPanel::deletePath, this, true));
  connect(path_following_buttons_.start_following_button, &QPushButton::clicked, this,
          std::bind(&NavigationControlPanel::operatePathFollowing, this, false));
  connect(path_following_buttons_.stop_following_button, &QPushButton::clicked, this,
          std::bind(&NavigationControlPanel::operatePathFollowing, this, true));
}

void NavigationControlPanel::deletePath(bool all)
{
  DeletePathService service;
  service.request.all = all;
  if(!delete_path_service_client_.call(service))
  {
    ROS_ERROR("Failed to call service delete_path");
  }
}

void NavigationControlPanel::generatePath()
{
  generatePath({0.0, 0.0, 0.0, 0.0, 0.0, 0});
}

void NavigationControlPanel::generatePath(const std::array<double, 6>& param)
{
  GeneratePathService service;
  service.request.path_resolution = param.at(0);
  service.request.arc_curvature = param.at(1);
  service.request.arc_angle = param.at(2);
  service.request.line_length = param.at(3);
  service.request.line_angle = param.at(4);
  service.request.path_direction = param.at(5);
  if(!generate_path_service_client_.call(service))
  {
    ROS_ERROR("Failed to call service generate_path");
  }
}

void NavigationControlPanel::generatePresetPath(int id)
{
  deletePath(true);

  const auto& preset_path = PRESET.at(id);
  for(auto itr = preset_path.begin(); itr != preset_path.end(); ++itr)
    generatePath(*itr);
}

void NavigationControlPanel::showPath()
{
  ShowEntirePathService entire_path_service;
  if(!show_entire_path_service_client_.call(entire_path_service))
  {
    ROS_ERROR("Failed to call service show_entire_path");
  }

  ShowTrajectoryService trajectory_service;
  if(!show_trajectory_service_client_.call(trajectory_service))
  {
    ROS_ERROR("Failed to call service show_trajectory");
  }
}

void NavigationControlPanel::operatePathFollowing(bool stop)
{
  OperatePathFollowingService service;
  service.request.stop = stop;
  if(!operate_path_following_service_client_.call(service))
  {
    ROS_ERROR("Failed to call service operate_path_following");
  }
}

} // namespace agv_navigation

PLUGINLIB_EXPORT_CLASS(agv_navigation::NavigationControlPanel, rviz::Panel);