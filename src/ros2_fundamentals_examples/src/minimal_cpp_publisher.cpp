#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalCppPublisher : public rclcpp::Node
{
public:
  MinimalCppPublisher()
  : Node("minimal_cpp_publisher"), counter_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("cpp_example_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MinimalCppPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::String message;
    message.data = "Hello World: " + std::to_string(counter_);
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: \"%s\"", message.data.c_str());
    counter_++;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalCppPublisher>());
  rclcpp::shutdown();
  return 0;
}
