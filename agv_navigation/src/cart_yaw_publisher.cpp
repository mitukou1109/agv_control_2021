#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <agv_control_lib/rviz_handler.h>
#include <laser_line_extraction/LineSegmentList.h>

class CartYawPublisher
{
public:

  CartYawPublisher()
  {
    ros::NodeHandle nh(""), pnh("~");

    cart_yaw_pub_ = nh.advertise<std_msgs::Float32>("cart_yaw", 10);
    cart_lines_sub_ = nh.subscribe("cart_lines", 10, &CartYawPublisher::cartLinesCB, this);

    pnh.param("publish_period", publish_period_, 0.1);
    pnh.param("reverse_sign", reverse_sign_, false);

    timer_ = nh.createTimer(ros::Duration(publish_period_), &CartYawPublisher::publish, this);
  }
  
private:
  
  // 角度差をpub
  void publish(const ros::TimerEvent& e)
  {
    ROS_INFO("cart_yaw[deg]: %f", cart_yaw_*180/M_PI);
    rh_.showOverlayText("cart_yaw[deg]: " + std::to_string(cart_yaw_*180/M_PI));
    
    std_msgs::Float32 cart_yaw_msg;
    cart_yaw_msg.data = cart_yaw_;
    cart_yaw_pub_.publish(cart_yaw_msg);
  }

  // トレーラの直線検出時に角度差を計算
  void cartLinesCB(const laser_line_extraction::LineSegmentList& msg)
  {
    auto num_of_lines = msg.line_segments.size();

    if(num_of_lines == 0)
    {
      ROS_WARN("No cart lines received");
      return;
    }

    // 直線が複数検出される場合があるので、それぞれに対して計算し候補を配列に
    std::vector<float> estimated_cart_yaws;
    for(auto line : msg.line_segments)
    {
      float estimated_cart_yaw = std::atan2(line.end.at(1)-line.start.at(1),
                                            line.end.at(0)-line.start.at(0)) - M_PI/2;
      if(estimated_cart_yaw < -M_PI) estimated_cart_yaw += 2*M_PI;
      estimated_cart_yaws.push_back(estimated_cart_yaw);
    }

    // 候補のうち現在の角度差と最も近いものを新しい角度差とする
    // (壁の誤検出時などに角度差が大きくぶれるのを防ぐため)
    cart_yaw_ = *std::min_element(
      estimated_cart_yaws.begin(), estimated_cart_yaws.end(),
      [this](float a, float b)
      {
        return std::abs(a-cart_yaw_) < std::abs(b-cart_yaw_);
      }
    );

    // 必要に応じて符号反転(LRFが下向きになっている場合など)
    if(reverse_sign_) cart_yaw_ *= -1;
  }

  ros::Timer timer_;

  ros::Publisher cart_yaw_pub_;
  ros::Subscriber cart_lines_sub_;
  
  RvizHandler rh_;

  float cart_yaw_;

  double publish_period_;

  bool reverse_sign_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cart_yaw_publisher");

  CartYawPublisher cart_yaw_publisher;

  ros::spin();

  return 0;
}