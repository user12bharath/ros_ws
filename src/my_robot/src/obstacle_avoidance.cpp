#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance()
  : Node("obstacle_avoidance"), safe_distance_(0.5)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node has been started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double max_range = msg->range_max;
    std::vector<float> clean_ranges;
    clean_ranges.reserve(msg->ranges.size());

    for (const float range : msg->ranges) {
      if (std::isinf(range) || std::isnan(range)) {
        clean_ranges.push_back(static_cast<float>(max_range));
      } else {
        clean_ranges.push_back(range);
      }
    }

    std::vector<float> front_ranges;
    const size_t n = clean_ranges.size();
    const size_t front_count = std::min<size_t>(30, n);

    for (size_t i = 0; i < front_count; ++i) {
      front_ranges.push_back(clean_ranges[i]);
    }
    const size_t back_start = std::min<size_t>(330, n);
    for (size_t i = back_start; i < n; ++i) {
      front_ranges.push_back(clean_ranges[i]);
    }

    const float min_front_distance = front_ranges.empty() ?
      static_cast<float>(max_range) :
      *std::min_element(front_ranges.begin(), front_ranges.end());

    geometry_msgs::msg::Twist twist_msg;
    if (min_front_distance < safe_distance_) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.5;
    } else {
      twist_msg.linear.x = 0.2;
      twist_msg.angular.z = 0.0;
    }

    publisher_->publish(twist_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "MIN FRONT DISTANCE: %.2fm, CMD_VEL: linear.x=%.2f, angular.z=%.2f",
      min_front_distance, twist_msg.linear.x, twist_msg.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  double safe_distance_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
