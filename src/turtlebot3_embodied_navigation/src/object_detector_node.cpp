// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "turtlebot3_embodied_navigation/perception_utils.hpp"

namespace turtlebot3_embodied_navigation
{

namespace
{

constexpr double kMaximumInputAgeMs = 500.0;

std::string shape_string(const std::vector<std::int64_t> & shape)
{
  std::ostringstream text;
  for (const auto dimension : shape) {
    text << (text.tellp() > 0 ? "x" : "") << dimension;
  }
  return text.str();
}

rmw_qos_profile_t latest_sensor_qos()
{
  auto qos = rmw_qos_profile_sensor_data;
  qos.depth = 1;
  return qos;
}

}  // namespace

class ObjectDetectorNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  ObjectDetectorNode()
  : Node("object_detector"),
    color_sub_(this, declare_parameter("color_topic", "/camera/color/image_raw"),
      latest_sensor_qos()),
    depth_sub_(this, declare_parameter("depth_topic", "/camera/depth/image_raw"),
      latest_sensor_qos()),
    sync_(SyncPolicy(2), color_sub_, depth_sub_)
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names", {"ball"});
    input_size_ = declare_parameter("input_size", 640);
    confidence_threshold_ = declare_parameter("confidence_threshold", 0.50);
    nms_threshold_ = declare_parameter("nms_threshold", 0.45);
    minimum_depth_ = declare_parameter("minimum_depth", 0.10);
    maximum_depth_ = declare_parameter("maximum_depth", 8.0);
    const bool publish_all_classes = declare_parameter("publish_all_classes", true);

    const auto hardware_threads = std::max(1U, std::thread::hardware_concurrency());
    rcl_interfaces::msg::ParameterDescriptor thread_descriptor;
    thread_descriptor.description =
      "ONNX Runtime intra-op thread count; fixed when the session is created";
    thread_descriptor.read_only = true;
    rcl_interfaces::msg::IntegerRange thread_range;
    thread_range.from_value = 1;
    thread_range.to_value = static_cast<std::int64_t>(hardware_threads);
    thread_range.step = 1;
    thread_descriptor.integer_range.push_back(thread_range);
    const auto default_threads = std::min<std::int64_t>(
      2, static_cast<std::int64_t>(hardware_threads));
    const auto intra_op_threads = declare_parameter<std::int64_t>(
      "intra_op_threads", default_threads, thread_descriptor);

    if (!publish_all_classes) {
      // Jazzy cannot infer a static parameter type from an empty initializer.
      const auto disabled_class_ids = declare_parameter<std::vector<std::int64_t>>(
        "disabled_class_ids", std::vector<std::int64_t>{});
      for (const auto class_id : disabled_class_ids) {
        if (class_id < 0 || class_id >= static_cast<std::int64_t>(class_names_.size())) {
          throw std::invalid_argument("disabled_class_ids contains an invalid class index");
        }
        disabled_class_ids_.insert(static_cast<int>(class_id));
      }
    }

    if (model_path_.empty() || !std::filesystem::is_regular_file(model_path_)) {
      throw std::runtime_error("model_path must reference a readable YOLO11 ONNX file");
    }
    session_options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetIntraOpNumThreads(static_cast<int>(intra_op_threads));
    session_options_.SetInterOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(
      ort_environment_, model_path_.c_str(), session_options_);
    Ort::AllocatorWithDefaultOptions allocator;
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      throw std::runtime_error("YOLO11 ONNX model must have one input and one output");
    }
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
    const auto input_type = session_->GetInputTypeInfo(0);
    const auto output_type = session_->GetOutputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    input_shape_ = input_info.GetShape();
    output_shape_ = output_info.GetShape();
    const std::vector<std::int64_t> expected_input{1, 3, input_size_, input_size_};
    const std::int64_t expected_channels = static_cast<std::int64_t>(class_names_.size() + 4);
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      input_shape_ != expected_input || output_shape_.size() != 3 ||
      output_shape_[0] != 1 || output_shape_[1] != expected_channels || output_shape_[2] <= 0)
    {
      throw std::runtime_error(
              "unexpected static YOLO11 ONNX tensor contract: input=" +
              shape_string(input_shape_) + ", output=" + shape_string(output_shape_) +
              ", classes=" + std::to_string(class_names_.size()));
    }

    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_ = std::move(message);
      });
    detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/embodied/detections", 10);
    raw_detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "/embodied/detections_2d", 10);
    debug_pub_ = create_publisher<Image>("/embodied/debug_image", 10);
    inference_pub_ = create_publisher<std_msgs::msg::Float32>("/embodied/inference_ms", 10);
    sync_.registerCallback(
      std::bind(&ObjectDetectorNode::on_images, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Loaded YOLO11 model: %s (ORT intra-op threads: %d/%u)",
      model_path_.c_str(), static_cast<int>(intra_op_threads), hardware_threads);
  }

private:
  void on_images(const Image::ConstSharedPtr & color_message, const Image::ConstSharedPtr & depth_message)
  {
    const double input_age_ms =
      (get_clock()->now() - rclcpp::Time(color_message->header.stamp)).seconds() * 1000.0;
    // 过期图像已失去导航价值，继续推理只会扩大桥接积压。
    if (input_age_ms > kMaximumInputAgeMs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping stale camera frame: %.1f ms old",
        input_age_ms);
      return;
    }

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
      const auto inference_start = std::chrono::steady_clock::now();
      const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      auto input = Ort::Value::CreateTensor<float>(
        memory, blob.ptr<float>(), blob.total(), input_shape_.data(), input_shape_.size());
      const std::array<const char *, 1> input_names{input_name_.c_str()};
      const std::array<const char *, 1> output_names{output_name_.c_str()};
      auto outputs = session_->Run(
        Ort::RunOptions{nullptr}, input_names.data(), &input, 1, output_names.data(), 1);
      if (outputs.size() != 1 || !outputs.front().IsTensor()) {
        throw std::runtime_error("ONNX Runtime returned an invalid YOLO11 output");
      }
      const auto actual_shape = outputs.front().GetTensorTypeAndShapeInfo().GetShape();
      if (actual_shape != output_shape_) {
        throw std::runtime_error("ONNX Runtime changed the static YOLO11 output shape");
      }
      const std::array<int, 3> cv_shape{
        static_cast<int>(output_shape_[0]), static_cast<int>(output_shape_[1]),
        static_cast<int>(output_shape_[2])};
      const cv::Mat output(
        static_cast<int>(cv_shape.size()), cv_shape.data(), CV_32F,
        outputs.front().GetTensorMutableData<float>());
      std_msgs::msg::Float32 inference;
      inference.data = std::chrono::duration<float, std::milli>(
        std::chrono::steady_clock::now() - inference_start).count();
      inference_pub_->publish(inference);
      publish_detections(
        color_message, bgr, depth, output, scale, padding_x, padding_y);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "YOLO11 inference failed: %s", error.what());
    }
  }

  void publish_detections(
    const Image::ConstSharedPtr & source, cv::Mat & bgr, const cv::Mat & depth,
    const cv::Mat & output, const double scale, const int padding_x, const int padding_y)
  {
    vision_msgs::msg::Detection3DArray array;
    array.header = source->header;
    vision_msgs::msg::Detection2DArray raw_array;
    raw_array.header = source->header;
    const auto detections = decode_yolo11(
      output, scale, bgr.size(), padding_x, padding_y, static_cast<int>(class_names_.size()),
      confidence_threshold_, nms_threshold_);

    for (const auto & candidate : detections) {
      if (disabled_class_ids_.count(candidate.class_index) != 0) {
        continue;
      }
      const std::string class_name = class_names_.at(candidate.class_index);

      // This topic deliberately precedes depth validation: acceptance must inspect every
      // YOLO result, including boxes whose depth cannot be projected into 3D.
      vision_msgs::msg::Detection2D raw_detection;
      raw_detection.header = raw_array.header;
      raw_detection.id = class_name;
      raw_detection.bbox.center.position.x = candidate.box.x + candidate.box.width * 0.5;
      raw_detection.bbox.center.position.y = candidate.box.y + candidate.box.height * 0.5;
      raw_detection.bbox.center.theta = 0.0;
      raw_detection.bbox.size_x = candidate.box.width;
      raw_detection.bbox.size_y = candidate.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose raw_hypothesis;
      raw_hypothesis.hypothesis.class_id = class_name;
      raw_hypothesis.hypothesis.score = candidate.confidence;
      raw_detection.results.push_back(std::move(raw_hypothesis));
      raw_array.detections.push_back(std::move(raw_detection));

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
      const std::string label = class_name + ":" + color;

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
    raw_detection_pub_->publish(raw_array);
    auto debug_message = cv_bridge::CvImage(source->header, "bgr8", bgr).toImageMsg();
    debug_pub_->publish(*debug_message);
  }

  std::string model_path_;
  std::vector<std::string> class_names_;
  std::set<int> disabled_class_ids_;
  int input_size_;
  double confidence_threshold_;
  double nms_threshold_;
  double minimum_depth_;
  double maximum_depth_;
  Ort::Env ort_environment_{ORT_LOGGING_LEVEL_WARNING, "object_detector"};
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<std::int64_t> input_shape_;
  std::vector<std::int64_t> output_shape_;

  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr raw_detection_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inference_pub_;
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
