#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

nav_msgs::msg::Path pathCarla;


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalSubscription : public rclcpp::Node
{
  public:

    MinimalSubscription()
    : Node("subscribe_pyth_node")
    {

      rclcpp::QoS qos(10);
      qos.keep_last(10);
      qos.best_effort();
      qos.durability_volatile();

      mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
                          "/carla/ego_vehicle/odometry", qos,
                          std::bind(&MinimalSubscription::odomCallback, this, _1) );

      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    }

  private:

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Camera position in map frame
        geometry_msgs::msg::PoseStamped carla_pose;
        carla_pose.pose.position.x = msg->pose.pose.position.x;
        carla_pose.pose.position.y = msg->pose.pose.position.y;
        carla_pose.pose.position.z = 0;
        pathCarla.poses.push_back(carla_pose);
        pathCarla.header.frame_id = "map";
        pathCarla.header.stamp = this->get_clock()->now();
        publisher_->publish(pathCarla);
        RCLCPP_INFO(get_logger(),"Received pose in %f", carla_pose.pose.position.x);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscription>());
    rclcpp::shutdown();
    return 0;
  }
