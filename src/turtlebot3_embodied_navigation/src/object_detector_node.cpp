// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "turtlebot3_embodied_navigation/perception_utils.hpp"

namespace turtlebot3_embodied_navigation
{

class ObjectDetectorNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  ObjectDetectorNode()
  : Node("object_detector"),
    color_sub_(this, declare_parameter("color_topic", "/camera/color/image_raw"),
      rmw_qos_profile_sensor_data),
    depth_sub_(this, declare_parameter("depth_topic", "/camera/depth/image_raw"),
      rmw_qos_profile_sensor_data),
    sync_(SyncPolicy(10), color_sub_, depth_sub_)
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names", {"cup", "bottle", "backpack", "ball", "box"});
    input_size_ = declare_parameter("input_size", 640);
    confidence_threshold_ = declare_parameter("confidence_threshold", 0.50);
    nms_threshold_ = declare_parameter("nms_threshold", 0.45);
    minimum_depth_ = declare_parameter("minimum_depth", 0.10);
    maximum_depth_ = declare_parameter("maximum_depth", 8.0);

    if (model_path_.empty() || !std::filesystem::is_regular_file(model_path_)) {
      throw std::runtime_error("model_path must reference a readable YOLO11 ONNX file");
    }
    net_ = cv::dnn::readNetFromONNX(model_path_);
    if (net_.empty()) {
      throw std::runtime_error("OpenCV failed to load YOLO11 ONNX model");
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_ = std::move(message);
      });
    detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/embodied/detections", 10);
    debug_pub_ = create_publisher<Image>("/embodied/debug_image", 10);
    sync_.registerCallback(
      std::bind(&ObjectDetectorNode::on_images, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Loaded YOLO11 model: %s", model_path_.c_str());
  }

private:
  void on_images(const Image::ConstSharedPtr & color_message, const Image::ConstSharedPtr & depth_message)
  {
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for CameraInfo");
      return;
    }

    cv::Mat bgr;
    cv::Mat depth;
    try {
      bgr = cv_bridge::toCvCopy(color_message, sensor_msgs::image_encodings::BGR8)->image;
      depth = cv_bridge::toCvCopy(depth_message, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "Image conversion failed: %s", error.what());
      return;
    }

    const double scale = std::min(
      static_cast<double>(input_size_) / bgr.cols,
      static_cast<double>(input_size_) / bgr.rows);
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(), scale, scale);
    cv::Mat padded(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
    const int padding_x = (input_size_ - resized.cols) / 2;
    const int padding_y = (input_size_ - resized.rows) / 2;
    resized.copyTo(padded(cv::Rect(padding_x, padding_y, resized.cols, resized.rows)));
    cv::Mat blob = cv::dnn::blobFromImage(
      padded, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false);

    try {
      net_.setInput(blob);
      const cv::Mat output = net_.forward();
      publish_detections(
        color_message, bgr, depth, output, scale, padding_x, padding_y);
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR(get_logger(), "YOLO11 inference failed: %s", error.what());
    }
  }

  void publish_detections(
    const Image::ConstSharedPtr & source, cv::Mat & bgr, const cv::Mat & depth,
    const cv::Mat & output, const double scale, const int padding_x, const int padding_y)
  {
    vision_msgs::msg::Detection3DArray array;
    array.header = source->header;
    const auto detections = decode_yolo11(
      output, scale, bgr.size(), padding_x, padding_y, static_cast<int>(class_names_.size()),
      confidence_threshold_, nms_threshold_);

    for (const auto & candidate : detections) {
      const float range = median_depth(
        depth, candidate.box, minimum_depth_, maximum_depth_);
      if (!std::isfinite(range)) {
        continue;
      }
      const double pixel_x = candidate.box.x + candidate.box.width * 0.5;
      const double pixel_y = candidate.box.y + candidate.box.height * 0.5;
      const Point3D point = project_pixel(
        pixel_x, pixel_y, range, camera_info_->k[0], camera_info_->k[4],
        camera_info_->k[2], camera_info_->k[5]);
      const std::string color = classify_color(bgr, candidate.box);
      const std::string label = class_names_.at(candidate.class_index) + ":" + color;

      vision_msgs::msg::Detection3D detection;
      detection.header = array.header;
      detection.id = label;
      detection.bbox.center.position.x = point.x;
      detection.bbox.center.position.y = point.y;
      detection.bbox.center.position.z = point.z;
      detection.bbox.center.orientation.w = 1.0;
      detection.bbox.size.x = candidate.box.width * range / camera_info_->k[0];
      detection.bbox.size.y = candidate.box.height * range / camera_info_->k[4];
      detection.bbox.size.z = 0.10;

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = label;
      hypothesis.hypothesis.score = candidate.confidence;
      hypothesis.pose.pose = detection.bbox.center;
      detection.results.push_back(std::move(hypothesis));
      array.detections.push_back(std::move(detection));

      cv::rectangle(bgr, candidate.box, cv::Scalar(0, 255, 0), 2);
      cv::putText(
        bgr, label + " " + cv::format("%.2f", candidate.confidence),
        cv::Point(candidate.box.x, std::max(18, candidate.box.y - 4)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    detection_pub_->publish(array);
    auto debug_message = cv_bridge::CvImage(source->header, "bgr8", bgr).toImageMsg();
    debug_pub_->publish(*debug_message);
  }

  std::string model_path_;
  std::vector<std::string> class_names_;
  int input_size_;
  double confidence_threshold_;
  double nms_threshold_;
  double minimum_depth_;
  double maximum_depth_;
  cv::dnn::Net net_;

  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
};

}  // namespace turtlebot3_embodied_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<turtlebot3_embodied_navigation::ObjectDetectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("object_detector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
