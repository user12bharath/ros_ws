#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class TargetReactor : public rclcpp::Node
{
public:
  TargetReactor()
  : Node("target_reactor"),
    target_visible_(false),
    front_distance_(std::numeric_limits<double>::infinity()),
    stop_distance_(1.4),
    slow_distance_(1.8),
    lost_count_(0),
    lost_threshold_(5)
  {
    alert_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/vision/sim_alert", 10,
      std::bind(&TargetReactor::alert_callback, this, std::placeholders::_1));

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&TargetReactor::scan_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_safety", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TargetReactor::decide_action, this));
  }

private:
  void alert_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "TARGET DETECTED") {
      target_visible_ = true;
      lost_count_ = 0;
    } else {
      lost_count_++;
      if (lost_count_ >= lost_threshold_) {
        target_visible_ = false;
      }
    }
  }

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

    front_distance_ = front_ranges.empty() ?
      max_range :
      static_cast<double>(*std::min_element(front_ranges.begin(), front_ranges.end()));
  }

  void decide_action()
  {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();

    if (!target_visible_) {
      twist.twist.linear.x = 0.2;
    } else if (front_distance_ > slow_distance_) {
      twist.twist.linear.x = 0.2;
    } else if (front_distance_ > stop_distance_) {
      const double scale =
        (front_distance_ - stop_distance_) / (slow_distance_ - stop_distance_);
      twist.twist.linear.x = 0.2 * scale;
    } else {
      twist.twist.linear.x = 0.0;
    }

    publisher_->publish(twist);

    RCLCPP_INFO(
      this->get_logger(),
      "Target visible: %s, Front distance: %.2f, Speed: %.2f",
      target_visible_ ? "true" : "false",
      front_distance_,
      twist.twist.linear.x);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alert_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool target_visible_;
  double front_distance_;
  double stop_distance_;
  double slow_distance_;
  int lost_count_;
  int lost_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetReactor>());
  rclcpp::shutdown();
  return 0;
}
