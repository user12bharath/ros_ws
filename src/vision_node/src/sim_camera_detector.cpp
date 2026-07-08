#include <memory>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class SimCameraDetector : public rclcpp::Node
{
public:
  SimCameraDetector()
  : Node("sim_camera_detector")
  {
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&SimCameraDetector::image_callback, this, std::placeholders::_1));

    count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/vision/sim_face_count", 10);
    alert_publisher_ = this->create_publisher<std_msgs::msg::String>("/vision/sim_alert", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "SimCameraDetector node has been started. subscribed to /camera/image_raw");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting ROS Image to OpenCV: %s", e.what());
      return;
    }

    cv::Mat cv_image = cv_ptr->image;

    cv::Mat hsv;
    cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

    const cv::Scalar lower_red1(0, 120, 70);
    const cv::Scalar upper_red1(10, 255, 255);
    const cv::Scalar lower_red2(170, 120, 70);
    const cv::Scalar upper_red2(180, 255, 255);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);

    cv::Mat mask;
    cv::bitwise_or(mask1, mask2, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    bool detected = false;
    double largest_area = 0.0;
    cv::Rect target_box;

    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area > 300.0 && area > largest_area) {
        largest_area = area;
        target_box = cv::boundingRect(contour);
        detected = true;
      }
    }

    if (detected) {
      cv::rectangle(cv_image, target_box, cv::Scalar(0, 255, 0), 2);
      cv::putText(
        cv_image,
        "Target area=" + std::to_string(static_cast<int>(largest_area)),
        cv::Point(target_box.x, std::max(target_box.y - 10, 0)),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    std_msgs::msg::String alert_msg;
    alert_msg.data = detected ? "TARGET DETECTED" : "CLEAR";
    alert_publisher_->publish(alert_msg);

    std_msgs::msg::Int32 count_msg;
    count_msg.data = detected ? 1 : 0;
    count_publisher_->publish(count_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "area of the detected target: %.0f, alert: %s",
      largest_area, alert_msg.data.c_str());

    cv::imshow("sim camera view", cv_image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimCameraDetector>());
  rclcpp::shutdown();
  return 0;
}
