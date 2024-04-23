// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Sensors : public rclcpp::Node
{
public:
  Sensors()
  : Node("sensor_subscriber")
  {
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 1, std::bind(&Sensors::gps_callback, this, _1));

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 1, std::bind(&Sensors::image_callback, this, _1));

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, std::bind(&Sensors::joy_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&Sensors::timer_callback, this));
  }

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix & msg) 
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f' at %d", msg.longitude, msg.header.stamp.nanosec);
    last_gps_ = msg;
  }
  void image_callback(const sensor_msgs::msg::Image & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I got an image at %d", msg.header.stamp.nanosec);
  }
  void joy_callback(const sensor_msgs::msg::Joy & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I got joy message button 0 = %d", msg.buttons[0]);
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d', long=%f", last_gps_.header.stamp.nanosec,last_gps_.longitude);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  sensor_msgs::msg::NavSatFix last_gps_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());
  rclcpp::shutdown();
  return 0;
}
