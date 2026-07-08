#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalCppSubscriber : public rclcpp::Node
{
public:
  MinimalCppSubscriber()
  : Node("minimal_cpp_subscriber")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "cpp_example_topic", 10,
      std::bind(&MinimalCppSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  void listener_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: \"%s\"", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalCppSubscriber>());
  rclcpp::shutdown();
  return 0;
}
