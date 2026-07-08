#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class YoloCameraDetector : public rclcpp::Node
{
public:
  YoloCameraDetector()
  : Node("yolo_camera_detector"), input_width_(640), input_height_(640)
  {
    this->declare_parameter<std::vector<std::string>>("target_classes", {"person"});
    this->declare_parameter<double>("confidence_threshold", 0.5);
    this->declare_parameter<std::string>("model_filename", "yolov8n.onnx");

    target_classes_ = this->get_parameter("target_classes").as_string_array();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    const std::string model_filename = this->get_parameter("model_filename").as_string();

    std::string model_path =
      ament_index_cpp::get_package_share_directory("vision_node") +
      "/data/" + model_filename;

    if (!std::filesystem::exists(model_path)) {
      model_path = model_filename;
    }

    if (!std::filesystem::exists(model_path)) {
      throw std::runtime_error(
        "YOLO model not found at " + model_path +
        ". Export yolov8n.pt to ONNX and place it in vision_node/data/yolov8n.onnx");
    }

    RCLCPP_INFO(this->get_logger(), "Loading %s ...", model_path.c_str());
    net_ = cv::dnn::readNetFromONNX(model_path);

    if (net_.empty()) {
      throw std::runtime_error("Failed to load ONNX model: " + model_path);
    }

    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    RCLCPP_INFO(
      this->get_logger(),
      "Model loaded. Watching for target classes with confidence threshold: %.2f",
      confidence_threshold_);

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&YoloCameraDetector::image_callback, this, std::placeholders::_1));

    count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/vision/sim_face_count", 10);
    alert_publisher_ = this->create_publisher<std_msgs::msg::String>("/vision/sim_alert", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Yolocameradetector node has been started. subscribed to /camera/image_raw");
  }

private:
  struct Detection
  {
    cv::Rect box;
    float confidence;
    int class_id;
  };

  std::vector<Detection> run_inference(const cv::Mat & image)
  {
    const int orig_w = image.cols;
    const int orig_h = image.rows;

    cv::Mat blob = cv::dnn::blobFromImage(
      image, 1.0 / 255.0, cv::Size(input_width_, input_height_),
      cv::Scalar(), true, false);

    net_.setInput(blob);

  // Raw output shape: [1, 84, 8400]
  // dims[0]=1 (batch), dims[1]=84 (4 box + 80 classes), dims[2]=8400 (proposals)
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    cv::Mat raw = outputs[0];  // shape [1, 84, 8400]

  // Reshape to [84, 8400], then transpose to [8400, 84]
  // so each row is one proposal: [cx, cy, w, h, class0..class79]
    cv::Mat output = raw.reshape(1, raw.size[1]);  // [84, 8400]
    cv::Mat output_t;
    cv::transpose(output, output_t);               // [8400, 84]

    const float x_scale = static_cast<float>(orig_w) / static_cast<float>(input_width_);
    const float y_scale = static_cast<float>(orig_h) / static_cast<float>(input_height_);

    std::vector<Detection> detections;
    for (int i = 0; i < output_t.rows; ++i) {
      const float * row = output_t.ptr<float>(i);
      const float cx = row[0];
      const float cy = row[1];
      const float w  = row[2];
      const float h  = row[3];

      float best_conf = 0.0f;
      int best_class = -1;
      for (int c = 4; c < output_t.cols; ++c) {
        if (row[c] > best_conf) {
          best_conf = row[c];
          best_class = c - 4;
        }
      }

      if (best_conf < static_cast<float>(confidence_threshold_) || best_class < 0) {
        continue;
      }

      const std::string class_name = get_class_name(best_class);
      if (!is_target_class(class_name)) {
        continue;
      }

      const int x = static_cast<int>((cx - w / 2.0f) * x_scale);
      const int y = static_cast<int>((cy - h / 2.0f) * y_scale);
      const int width  = static_cast<int>(w * x_scale);
      const int height = static_cast<int>(h * y_scale);

      Detection detection;
      detection.box = cv::Rect(x, y, width, height);
      detection.confidence = best_conf;
      detection.class_id = best_class;
      detections.push_back(detection);
    }

    return detections;
  }



  bool is_target_class(const std::string & class_name) const
  {
    return std::find(target_classes_.begin(), target_classes_.end(), class_name) !=
           target_classes_.end();
  }

  std::string get_class_name(int class_id) const
  {
    static const std::vector<std::string> coco_names = {
      "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
      "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
      "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
      "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
      "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
      "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
      "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
      "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
      "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
      "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };

    if (class_id >= 0 && class_id < static_cast<int>(coco_names.size())) {
      return coco_names[static_cast<size_t>(class_id)];
    }
    return "unknown";
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting ROS Image to Opencv: %s", e.what());
      return;
    }

    cv::Mat cv_image = cv_ptr->image;
    const auto detections = run_inference(cv_image);

    bool detected = false;
    double largest_area = 0.0;
    cv::Rect target_box;

    for (const auto & detection : detections) {
      const double area = static_cast<double>(detection.box.area());
      if (area > 300.0 && area > largest_area) {
        largest_area = area;
        target_box = detection.box;
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

    cv::imshow("yolo camera view", cv_image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_publisher_;

  cv::dnn::Net net_;
  std::vector<std::string> target_classes_;
  double confidence_threshold_;
  int input_width_;
  int input_height_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloCameraDetector>());
  rclcpp::shutdown();
  return 0;
}
