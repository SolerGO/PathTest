#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

float x1 = 23.5;
float x2 = -44.5;

float y2 = 4.5;
float y3 = 84.5;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path_test", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      nav_msgs::msg::Path message;
      geometry_msgs::msg::PoseStamped msg;
      for (float i = x1; i >=x2 ; i=i-0.2) {
        msg.pose.position.x = i;
        msg.pose.position.y = y2;
        msg.pose.position.z = 0;
        message.poses.push_back(msg);
      }
      for (float i = y2; i <=y3 ; i=i+0.2) {
        msg.pose.position.x = x2;
        msg.pose.position.y = i;
        msg.pose.position.z = 0;
        message.poses.push_back(msg);
      }
      for (float i = x2; i <=x1 ; i=i+0.2) {
        msg.pose.position.x = i;
        msg.pose.position.y = y3;
        msg.pose.position.z = 0;
        message.poses.push_back(msg);
      }
      for (float i = y3; i >=y2 ; i=i-0.2) {
        msg.pose.position.x = x1;
        msg.pose.position.y = i;
        msg.pose.position.z = 0;
        message.poses.push_back(msg);
      }
      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "map";
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
