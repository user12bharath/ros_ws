#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class VisionMonitor : public rclcpp::Node
{
public:
  VisionMonitor()
  : Node("vision_monitor"),
    latest_fps_(0.0),
    latest_face_count_(0),
    latest_alert_("No alerts")
  {
    face_count_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "/vision/face_count", 10,
      std::bind(&VisionMonitor::face_count_callback, this, std::placeholders::_1));

    fps_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/vision/fps", 10,
      std::bind(&VisionMonitor::fps_callback, this, std::placeholders::_1));

    alert_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/vision/alerts", 10,
      std::bind(&VisionMonitor::alert_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&VisionMonitor::print_status, this));
  }

private:
  void fps_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_fps_ = msg->data;
  }

  void face_count_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    latest_face_count_ = msg->data;
  }

  void alert_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_alert_ = msg->data;
  }

  void print_status()
  {
    RCLCPP_INFO(
      this->get_logger(),
      " [VISION STATUS] FPS: %.2f | Faces: %d | Status: %s",
      latest_fps_,
      latest_face_count_,
      latest_alert_.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr face_count_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fps_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alert_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  double latest_fps_;
  int latest_face_count_;
  std::string latest_alert_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionMonitor>());
  rclcpp::shutdown();
  return 0;
}
