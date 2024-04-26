// This code is for HW03

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"


using std::placeholders::_1;

class Sensors : public rclcpp::Node
{
public:
  Sensors()
  : Node("sensor_subscriber")
  {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 1, std::bind(&Sensors::image_callback, this, _1));

    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 1, std::bind(&Sensors::gps_callback, this, _1));

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, std::bind(&Sensors::joy_callback, this, _1));
  }


private:
  void image_callback(const sensor_msgs::msg::Image & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[IMAGE]: I got an image at device time [ns]: '%d'", msg.header.stamp.nanosec);
  }
  void gps_callback(const sensor_msgs::msg::NavSatFix & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[GPS]: I got GPS location at lat: '%f' and lon: '%f' at device time [ns]: '%d'", msg.latitude, msg.longitude, msg.header.stamp.nanosec);
  }
  void joy_callback(const sensor_msgs::msg::Joy & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[JOY]: I got joy message button 0 = %d",msg.buttons[0]);
  }


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());
  rclcpp::shutdown();
  return 0;
}
