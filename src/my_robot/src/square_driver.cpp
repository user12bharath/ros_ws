#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class SquareDriver : public rclcpp::Node
{
public:
  SquareDriver()
  : Node("square_driver", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})), counter_(0), side_(0), state_(0)
  {

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_out", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SquareDriver::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Starting outer perimeter mapping routing...");
  }

private:
  geometry_msgs::msg::TwistStamped make_twist(double linear_x = 0.0, double angular_z = 0.0)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;
    return msg;
  }

  void timer_callback()
  {
    if (state_ == 0) {
      if (counter_ < 150) {
        publisher_->publish(make_twist(0.2, 0.0));
        counter_++;
      } else {
        state_ = 1;
        counter_ = 0;
      }
    } else if (state_ == 1) {
      if (counter_ < 39) {
        publisher_->publish(make_twist(0.0, 0.4));
        counter_++;
      } else {
        state_ = 2;
        counter_ = 0;
      }
    } else if (state_ == 2) {
      if (counter_ < 150) {
        publisher_->publish(make_twist(0.2, 0.0));
        counter_++;
      } else {
        state_ = 3;
        counter_ = 0;
      }
    } else if (state_ == 3) {
      if (counter_ < 39) {
        publisher_->publish(make_twist(0.0, 0.4));
        counter_++;
      } else {
        state_ = 4;
        counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "Perimeter reached. Beginning 6m x 6m mapping loop.");
      }
    } else if (state_ == 4) {
      if (counter_ < 300) {
        publisher_->publish(make_twist(0.2, 0.0));
        counter_++;
      } else {
        state_ = 5;
        counter_ = 0;
      }
    } else if (state_ == 5) {
      if (counter_ < 39) {
        publisher_->publish(make_twist(0.0, 0.4));
        counter_++;
      } else {
        side_++;
        counter_ = 0;
        if (side_ < 4) {
          state_ = 4;
        } else {
          state_ = 6;
          RCLCPP_INFO(this->get_logger(), "Map complete! Standing still for Map Saver.");
        }
      }
    } else if (state_ == 6) {
      publisher_->publish(make_twist(0.0, 0.0));
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
  int side_;
  int state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareDriver>());
  rclcpp::shutdown();
  return 0;
}
