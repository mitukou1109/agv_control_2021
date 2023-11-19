// 没

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>

#include <agv_control_msgs/JackknifePreventionConfig.h>

class JackknifePrevention
{
public:
  JackknifePrevention()
  {
    ros::NodeHandle nh, pnh("~");
    cart_yaw_sub_ = nh.subscribe("cart/yaw", 10, &JackknifePrevention::cartYawCallback, this);
    base_vel_sub_ = nh.subscribe("base_vel", 10, &JackknifePrevention::baseVelCallback, this);
    cmd_vel_pub_  = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    yaw_thresh_ = pnh.param("yaw_thresh", 0.0) * M_PI / 180;
    Kp_ = pnh.param("Kp", 0.0);
    Ki_ = pnh.param("Ki", 0.0);
    Kd_ = pnh.param("Kd", 0.0);

    callback_ = boost::bind(&JackknifePrevention::dynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(callback_);
  }
  
private:

  void cartYawCallback(const std_msgs::Float32& msg)
  {
    cart_yaw_ = msg.data;
  }

  void baseVelCallback(const geometry_msgs::Twist& base_vel)
  {
    static float last_yaw_err, yaw_err_sum;
    static ros::Time last_ctrl_time = ros::Time::now();

    float yaw_target_abs = 0.0; // yaw_thresh_;
    geometry_msgs::Twist comp_vel;
    geometry_msgs::Twist cmd_vel = base_vel;

    double dt = (ros::Time::now() - last_ctrl_time).toSec();

    if(dt > 0.001 && std::fabs(cart_yaw_) > yaw_thresh_)
    {
      float yaw_err = cart_yaw_ - std::copysign(yaw_target_abs, cart_yaw_);
      yaw_err_sum += yaw_err*dt;
      comp_vel.angular.z =
        yaw_err*Kp_ + yaw_err_sum*Ki_ + (yaw_err-last_yaw_err)/dt*Kd_;
      last_yaw_err = yaw_err;
    }
    else
    {
      yaw_err_sum = 0;
    }

    cmd_vel.angular.z += comp_vel.angular.z;

    // if(std::fabs(cart_yaw_) > 55/180.*M_PI)
    // {
    //   ROS_INFO("Jackknife occurred");
    //   geometry_msgs::Twist zero_vel;
    //   cmd_vel = zero_vel;
    // }

    ROS_INFO("comp_vel: %f[m/s], %f[rad/s]", comp_vel.linear.x, comp_vel.angular.z);
    ROS_INFO("cmd_vel: %f[m/s], %f[rad/s]", cmd_vel.linear.x, cmd_vel.angular.z);

    cmd_vel_pub_.publish(cmd_vel);

    last_ctrl_time = ros::Time::now();
  }

  void dynamicReconfigureCallback(agv_navigation::JackknifePreventionConfig &config, uint32_t level)
  {
    Kp_ = config.Kp;
    Ki_ = config.Ki;
    Kd_ = config.Kd;
  }

  ros::Subscriber cart_yaw_sub_, base_vel_sub_;

  ros::Publisher cmd_vel_pub_;

  dynamic_reconfigure::Server<agv_navigation::JackknifePreventionConfig> server_;

  dynamic_reconfigure::Server<agv_navigation::JackknifePreventionConfig>::CallbackType callback_;

  float cart_yaw_;

  float yaw_thresh_;

  float Kp_, Ki_, Kd_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jackknife_prevention");

  JackknifePrevention jackknife_prevention();

  ros::spin();
  
  return 0;
}
