#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class FaceDetectorNode : public rclcpp::Node
{
public:
  FaceDetectorNode()
  : Node("face_detector"), running_(true), has_frame_(false)
  {
    face_count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/vision/face_count", 10);
    fps_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/vision/fps", 10);
    alert_publisher_ = this->create_publisher<std_msgs::msg::String>("/vision/alerts", 10);

    cap_.open(0);
    if (!cap_.isOpened()) {
      throw std::runtime_error("Failed to open webcam device 0");
    }

    const std::string cascade_path =
      ament_index_cpp::get_package_share_directory("vision_node") +
      "/data/haarcascade_frontalface_default.xml";

    if (!face_cascade_.load(cascade_path)) {
      throw std::runtime_error(
        "Cascade classifier failed to load. Check data files in CMakeLists.txt.");
    }

    prev_time_ = std::chrono::steady_clock::now();

    capture_thread_ = std::thread(&FaceDetectorNode::capture_loop, this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&FaceDetectorNode::process_frame, this));

    RCLCPP_INFO(this->get_logger(), "FaceDetector node started");
  }

  ~FaceDetectorNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    cap_.release();
    cv::destroyAllWindows();
  }

private:
  void capture_loop()
  {
    while (running_ && rclcpp::ok()) {
      cv::Mat frame;
      if (cap_.read(frame) && !frame.empty()) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = frame.clone();
        has_frame_ = true;
      }
    }
  }

  void process_frame()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!has_frame_) {
        return;
      }
      frame = latest_frame_.clone();
    }

    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(current_time - prev_time_).count();
    prev_time_ = current_time;
    const double fps = elapsed > 0.0 ? 1.0 / elapsed : 0.0;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(gray, faces, 1.1, 5, 0, cv::Size(30, 30));

    for (const auto & face : faces) {
      cv::rectangle(frame, face, cv::Scalar(0, 0, 255), 2);
    }

    cv::putText(
      frame, "FPS: " + std::to_string(fps), cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    cv::putText(
      frame, "Faces: " + std::to_string(faces.size()), cv::Point(10, 70),
      cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    std_msgs::msg::Int32 face_count_msg;
    face_count_msg.data = static_cast<int32_t>(faces.size());
    face_count_publisher_->publish(face_count_msg);

    std_msgs::msg::Float64 fps_msg;
    fps_msg.data = fps;
    fps_publisher_->publish(fps_msg);

    std_msgs::msg::String alert_msg;
    alert_msg.data = faces.empty() ? "NO FACES DETECTED" : "FACE DETECTED";
    alert_publisher_->publish(alert_msg);

    cv::imshow("face detection", frame);
    cv::waitKey(1);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr face_count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fps_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::VideoCapture cap_;
  cv::CascadeClassifier face_cascade_;
  cv::Mat latest_frame_;
  std::mutex frame_mutex_;
  std::thread capture_thread_;
  std::atomic<bool> running_;
  bool has_frame_;
  std::chrono::steady_clock::time_point prev_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
