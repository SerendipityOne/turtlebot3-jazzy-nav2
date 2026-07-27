// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "turtlebot3_gazebo/dataset_utils.hpp"

namespace turtlebot3_gazebo
{

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace std::chrono_literals;

constexpr std::size_t kWindowSize = 15;
constexpr std::size_t kRequiredMatches = 7;
constexpr double kMinimumDetectionRateHz = 5.0;
constexpr double kMaximumInferenceP95Ms = 200.0;
constexpr double kCameraForwardOffset = 0.08;
constexpr double kRobotZ = 0.01;
constexpr double kPlatformTopZ = 0.22;
constexpr double kPi = 3.14159265358979323846;

struct AssetRef
{
  std::string id;
  std::string target_class;
  int class_label;
  std::string surface;
  double z_offset;
  fs::path sdf_path;
};

struct RobotPose
{
  double x;
  double y;
  double yaw;
};

struct CaseSpec
{
  std::size_t index;
  std::string id;
  std::string world;
  std::string mode;
  std::uint64_t scene_seed;
  bool is_background;
  double distance;
  RobotPose robot_pose;
  std::optional<AssetRef> primary;
  std::vector<AssetRef> secondary;
};

struct FrameRecord
{
  vision_msgs::msg::Detection2DArray::ConstSharedPtr detections;
  std::chrono::steady_clock::time_point received_at;
};

struct LabelFrame
{
  std::vector<std::uint8_t> data;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
};

double percentile95(std::vector<float> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::ceil(0.95 * static_cast<double>(values.size()))) - 1;
  return values.at(index);
}

std::string detection_class(const std::string & id)
{
  const auto separator = id.find(':');
  return id.substr(0, separator);
}

class FinalPerceptionAcceptance : public rclcpp::Node
{
public:
  FinalPerceptionAcceptance()
  : Node("final_perception_acceptance")
  {
    output_dir_ = fs::absolute(declare_parameter<std::string>("output_dir"));
    holdout_manifest_path_ = fs::absolute(declare_parameter<std::string>("holdout_manifest"));
    world_name_ = declare_parameter<std::string>("world_name", "turtlebot3_house");
    case_world_ = declare_parameter<std::string>("case_world", world_name_);
    target_classes_ = declare_parameter<std::vector<std::string>>(
      "target_classes", {"ball"});
    robot_name_ = declare_parameter<std::string>("robot_name", "waffle_pi_cam");
    label_topic_ = declare_parameter<std::string>(
      "label_topic", "/dataset/segmentation/labels_map");
    preview_ = declare_parameter("preview", false);
    case_index_ = declare_parameter("case_index", -1);
    settle_frames_ = declare_parameter("settle_frames", 10);
    case_timeout_ = declare_parameter("case_timeout", 30.0);
    const std::set<std::string> unique_target_classes(
      target_classes_.begin(), target_classes_.end());
    if (output_dir_.empty() || holdout_manifest_path_.empty() || world_name_.empty() ||
      robot_name_.empty() || case_world_.empty() || target_classes_.empty() ||
      unique_target_classes.size() != target_classes_.size() || settle_frames_ <= 0 ||
      case_timeout_ <= 0.0 ||
      case_index_ < -1)
    {
      throw std::invalid_argument("final perception acceptance parameters are invalid");
    }
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/embodied/detections_2d", 10,
      std::bind(&FinalPerceptionAcceptance::on_detections, this, std::placeholders::_1));
    debug_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/embodied/debug_image", 10,
      std::bind(&FinalPerceptionAcceptance::on_debug_image, this, std::placeholders::_1));
    inference_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/embodied/inference_ms", 10,
      std::bind(&FinalPerceptionAcceptance::on_inference, this, std::placeholders::_1));
    if (!gz_node_.Subscribe(label_topic_, &FinalPerceptionAcceptance::on_labels, this)) {
      throw std::runtime_error("failed to subscribe to Gazebo label topic: " + label_topic_);
    }
  }

  void run()
  {
    prepare_output();
    const auto all_cases = load_cases();
    std::vector<CaseSpec> selected_cases;
    for (const auto & test_case : all_cases) {
      if (test_case.world == case_world_) {
        selected_cases.push_back(test_case);
      }
    }
    if (selected_cases.empty()) {
      throw std::runtime_error("holdout manifest has no cases for case_world: " + case_world_);
    }
    if (case_index_ >= static_cast<int>(selected_cases.size())) {
      throw std::invalid_argument("case_index is outside the selected world cases");
    }
    wait_for_infrastructure();
    if (preview_ || case_index_ >= 0) {
      selected_cases = {selected_cases.at(static_cast<std::size_t>(
          case_index_ >= 0 ? case_index_ : 0))};
    }

    json reports = json::array();
    for (const auto & test_case : selected_cases) {
      try {
        configure_scene(test_case);
        reports.push_back(run_case(test_case));
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "case %s failed: %s", test_case.id.c_str(), error.what());
        reports.push_back(failed_case_report(test_case, error.what()));
      }
    }
    remove_spawned_entities();
    const bool scope_passed = write_report(reports, selected_cases.size());
    if (!scope_passed) {
      RCLCPP_WARN(
        get_logger(), "world scope %s has failed cases; aggregate both world reports before deciding",
        case_world_.c_str());
    }
  }

private:
  void on_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++detection_sequence_;
    if (collecting_) {
      frames_.push_back({message, std::chrono::steady_clock::now()});
    }
    data_condition_.notify_all();
  }

  void on_debug_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_debug_image_ = message;
  }

  void on_inference(const std_msgs::msg::Float32::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++inference_sequence_;
    if (collecting_) {
      inference_ms_.push_back(message->data);
    }
    data_condition_.notify_all();
  }

  void on_labels(const gz::msgs::Image & message)
  {
    if (message.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Gazebo labels must use RGB_INT8");
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    labels_.data.assign(message.data().begin(), message.data().end());
    labels_.width = message.width();
    labels_.height = message.height();
    labels_.step = message.step();
    ++label_sequence_;
    data_condition_.notify_all();
  }

  static AssetRef parse_asset(const json & value, const fs::path & asset_root)
  {
    const auto raw_path = value.at("sdf_path").get<std::string>();
    const fs::path sdf_path = asset_root / raw_path;
    if (!fs::is_regular_file(sdf_path)) {
      throw std::runtime_error("holdout asset does not exist: " + sdf_path.string());
    }
    return {
      value.at("asset_id").get<std::string>(),
      value.at("target_class").get<std::string>(),
      std::stoi(value.at("class_id").get<std::string>()),
      value.at("surface").get<std::string>(),
      std::stod(value.at("z_offset").get<std::string>()),
      sdf_path};
  }

  std::vector<CaseSpec> load_cases()
  {
    std::ifstream input(holdout_manifest_path_);
    if (!input) {
      throw std::runtime_error("failed to open holdout manifest: " + holdout_manifest_path_.string());
    }
    json manifest;
    input >> manifest;
    if (manifest.value("schema_version", 0) != 1 ||
      manifest.value("classes", std::vector<std::string>{}) != target_classes_)
    {
      throw std::runtime_error("holdout manifest does not match the configured target classes");
    }
    const auto & positive = manifest.at("positive_acceptance");
    const auto & background = manifest.at("background_acceptance");
    if (positive.value("count", 0) != static_cast<int>(20 * target_classes_.size()) ||
      positive.value("per_class", 0) != 20 ||
      positive.value("minimum_iou", 0.0) != 0.5 || positive.value("minimum_recall", 0.0) != 0.9 ||
      background.value("count", 0) != 100 || background.value("raw_frame_false_positives", -1) != 0)
    {
      throw std::runtime_error("holdout manifest acceptance thresholds are invalid");
    }
    const fs::path asset_manifest = fs::absolute(manifest.at("asset_manifest").get<std::string>());
    if (!fs::is_regular_file(asset_manifest)) {
      throw std::runtime_error("holdout asset manifest does not exist: " + asset_manifest.string());
    }
    const fs::path asset_root = asset_manifest.parent_path();
    std::vector<CaseSpec> cases;
    std::map<std::string, std::size_t> positive_per_class;
    for (const auto & value : positive.at("cases")) {
      const auto primary = parse_asset(value.at("primary"), asset_root);
      const double distance = value.at("distance_m").get<double>();
      if (distance <= 0.0) {
        throw std::runtime_error("positive holdout distance must be positive");
      }
      std::vector<AssetRef> secondary;
      for (const auto & item : value.at("secondary")) {
        secondary.push_back(parse_asset(item, asset_root));
      }
      const double yaw = value.at("robot_yaw_rad").get<double>();
      cases.push_back({
        cases.size(), value.at("id").get<std::string>(), value.at("world").get<std::string>(),
        value.at("mode").get<std::string>(), value.at("scene_seed").get<std::uint64_t>(), false,
        distance, {-2.0, 0.5, yaw}, primary, std::move(secondary)});
      positive_per_class[primary.target_class]++;
    }
    for (const auto & name : target_classes_) {
      if (positive_per_class[name] != 20) {
        throw std::runtime_error("holdout must contain 20 positive cases for each class");
      }
    }
    for (const auto & value : background.at("cases")) {
      const auto & pose = value.at("robot_pose");
      std::vector<AssetRef> non_target;
      for (const auto & item : value.at("non_target")) {
        non_target.push_back(parse_asset(item, asset_root));
      }
      cases.push_back({
        cases.size(), value.at("id").get<std::string>(), value.at("world").get<std::string>(),
        value.at("background_mode").get<std::string>(), value.at("scene_seed").get<std::uint64_t>(), true,
        0.0, {pose.at("x").get<double>(), pose.at("y").get<double>(), pose.at("yaw_rad").get<double>()},
        std::nullopt, std::move(non_target)});
    }
    if (cases.size() != 20 * target_classes_.size() + 100) {
      throw std::runtime_error("holdout manifest case count does not match the configured target classes");
    }
    return cases;
  }

  std::string service(const std::string & name) const
  {
    return "/world/" + world_name_ + "/" + name;
  }

  template<typename Request>
  void request_boolean(const std::string & service_name, Request & request) const
  {
    gz::msgs::Boolean response;
    bool service_result = false;
    const bool executed = gz_node_.Request(service_name, request, 10000, response, service_result);
    if (!executed || !service_result || !response.data()) {
      throw std::runtime_error("Gazebo service failed: " + service_name);
    }
  }

  void wait_for_infrastructure()
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const bool ready = data_condition_.wait_for(lock, 30s, [this]() {
      return detection_sequence_ > 0 && inference_sequence_ > 0 && label_sequence_ > 0;
    });
    if (!ready) {
      throw std::runtime_error("timed out waiting for detector or segmentation label stream");
    }
  }

  void set_robot_pose(const RobotPose & pose) const
  {
    gz::msgs::Pose request;
    request.set_name(robot_name_);
    request.mutable_position()->set_x(pose.x);
    request.mutable_position()->set_y(pose.y);
    request.mutable_position()->set_z(kRobotZ);
    request.mutable_orientation()->set_z(std::sin(pose.yaw / 2.0));
    request.mutable_orientation()->set_w(std::cos(pose.yaw / 2.0));
    request_boolean(service("set_pose"), request);
  }

  void spawn_file(
    const std::string & name, const AssetRef & asset, const double x, const double y,
    const double z, const double yaw)
  {
    gz::msgs::EntityFactory request;
    request.set_name(name);
    request.set_allow_renaming(false);
    request.set_sdf_filename(asset.sdf_path.string());
    request.mutable_pose()->mutable_position()->set_x(x);
    request.mutable_pose()->mutable_position()->set_y(y);
    request.mutable_pose()->mutable_position()->set_z(z + asset.z_offset);
    request.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2.0));
    request_boolean(service("create"), request);
    spawned_entities_.push_back(name);
  }

  void spawn_platform(const std::string & name, const double x, const double y)
  {
    const std::string sdf =
      "<sdf version='1.8'><model name='acceptance_platform'><static>true</static>"
      "<pose>" + std::to_string(x) + " " + std::to_string(y) + " 0.11 0 0 0</pose>"
      "<link name='link'><collision name='collision'><geometry><box><size>0.40 0.40 0.22"
      "</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.40 "
      "0.40 0.22</size></box></geometry><material><diffuse>0.55 0.55 0.55 1</diffuse>"
      "</material></visual></link></model></sdf>";
    gz::msgs::EntityFactory request;
    request.set_name(name);
    request.set_allow_renaming(false);
    request.set_sdf(sdf);
    request_boolean(service("create"), request);
    spawned_entities_.push_back(name);
  }

  void remove_spawned_entities()
  {
    for (const auto & name : spawned_entities_) {
      gz::msgs::Entity request;
      request.set_name(name);
      request.set_type(gz::msgs::Entity::MODEL);
      request_boolean(service("remove"), request);
    }
    spawned_entities_.clear();
  }

  static std::array<double, 2> forward_position(
    const RobotPose & pose, const double distance, const double lateral = 0.0)
  {
    const double forward = kCameraForwardOffset + distance;
    return {
      pose.x + std::cos(pose.yaw) * forward - std::sin(pose.yaw) * lateral,
      pose.y + std::sin(pose.yaw) * forward + std::cos(pose.yaw) * lateral};
  }

  void spawn_at(
    const std::string & name, const AssetRef & asset, const std::array<double, 2> & position,
    const double yaw)
  {
    if (asset.surface == "raised") {
      spawn_platform(name + "_platform", position[0], position[1]);
    }
    spawn_file(
      name, asset, position[0], position[1], asset.surface == "raised" ? kPlatformTopZ : 0.02,
      yaw + kPi);
  }

  void configure_scene(const CaseSpec & test_case)
  {
    remove_spawned_entities();
    set_robot_pose(test_case.robot_pose);
    if (test_case.is_background) {
      for (std::size_t index = 0; index < test_case.secondary.size(); ++index) {
        spawn_at(
          "final_background_" + std::to_string(test_case.index) + "_" + std::to_string(index),
          test_case.secondary[index], forward_position(test_case.robot_pose, 1.25), test_case.robot_pose.yaw);
      }
      return;
    }
    const auto target = forward_position(test_case.robot_pose, test_case.distance);
    spawn_at(
      "final_primary_" + std::to_string(test_case.index), *test_case.primary, target,
      test_case.robot_pose.yaw);
    for (std::size_t index = 0; index < test_case.secondary.size(); ++index) {
      // The manifest fixes asset identities; this fixed layout keeps the primary unoccluded.
      const double lateral = (index % 2 == 0 ? 1.0 : -1.0) * (0.34 + 0.08 * index);
      const auto secondary_position = forward_position(
        test_case.robot_pose, test_case.distance + 0.20 * (index + 1), lateral);
      spawn_at(
        "final_secondary_" + std::to_string(test_case.index) + "_" + std::to_string(index),
        test_case.secondary[index], secondary_position, test_case.robot_pose.yaw);
    }
  }

  void collect_case_frames(
    std::vector<FrameRecord> & frames, std::vector<float> & inference, LabelFrame & labels,
    sensor_msgs::msg::Image::ConstSharedPtr & debug_image)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const auto detection_target = detection_sequence_ + static_cast<std::uint64_t>(settle_frames_);
    const auto label_target = label_sequence_ + static_cast<std::uint64_t>(settle_frames_);
    const bool settled = data_condition_.wait_for(
      lock, std::chrono::duration<double>(case_timeout_), [this, detection_target, label_target]() {
        return detection_sequence_ >= detection_target && label_sequence_ >= label_target;
      });
    if (!settled) {
      throw std::runtime_error("timed out while settling detector and segmentation streams");
    }
    frames_.clear();
    inference_ms_.clear();
    collecting_ = true;
    const bool collected = data_condition_.wait_for(
      lock, std::chrono::duration<double>(case_timeout_), [this]() {
        return frames_.size() >= kWindowSize && inference_ms_.size() >= kWindowSize;
      });
    collecting_ = false;
    if (!collected) {
      throw std::runtime_error("timed out collecting the 15-frame acceptance window");
    }
    frames.assign(frames_.begin(), frames_.begin() + kWindowSize);
    inference.assign(inference_ms_.begin(), inference_ms_.begin() + kWindowSize);
    labels = labels_;
    debug_image = latest_debug_image_;
  }

  static PixelBox expected_box(const LabelFrame & labels, const int class_label)
  {
    const auto boxes = decode_instance_boxes(
      labels.data, labels.width, labels.height, labels.step, 16, 4, 5);
    const auto selected = std::max_element(
      boxes.begin(), boxes.end(), [class_label](const auto & left, const auto & right) {
        const bool left_matches = left.class_id == class_label;
        const bool right_matches = right.class_id == class_label;
        if (left_matches != right_matches) {
          return !left_matches;
        }
        return left.visible_pixels < right.visible_pixels;
      });
    if (selected == boxes.end() || selected->class_id != class_label) {
      throw std::runtime_error("expected primary instance is not visible in the segmentation label map");
    }
    return {
      static_cast<double>(selected->min_x), static_cast<double>(selected->min_y),
      static_cast<double>(selected->max_x - selected->min_x + 1),
      static_cast<double>(selected->max_y - selected->min_y + 1)};
  }

  static PixelBox detection_box(const vision_msgs::msg::Detection2D & detection)
  {
    return {
      detection.bbox.center.position.x - detection.bbox.size_x * 0.5,
      detection.bbox.center.position.y - detection.bbox.size_y * 0.5,
      detection.bbox.size_x, detection.bbox.size_y};
  }

  json run_case(const CaseSpec & test_case)
  {
    std::vector<FrameRecord> frames;
    std::vector<float> inference;
    LabelFrame labels;
    sensor_msgs::msg::Image::ConstSharedPtr debug_image;
    collect_case_frames(frames, inference, labels, debug_image);

    const std::string expected = test_case.is_background ? "" : test_case.primary->target_class;
    std::optional<PixelBox> truth;
    if (!test_case.is_background) {
      truth = expected_box(labels, test_case.primary->class_label);
    }
    std::size_t iou_matches = 0;
    std::size_t raw_detections = 0;
    double best_iou = 0.0;
    std::map<std::string, std::size_t> raw_class_frames;
    for (const auto & frame : frames) {
      double frame_best_iou = 0.0;
      for (const auto & detection : frame.detections->detections) {
        ++raw_detections;
        const auto detected_class = detection_class(detection.id);
        ++raw_class_frames[detected_class];
        if (truth && detected_class == expected) {
          frame_best_iou = std::max(frame_best_iou, intersection_over_union(
              *truth, detection_box(detection)));
        }
      }
      best_iou = std::max(best_iou, frame_best_iou);
      if (truth && frame_best_iou >= 0.5) {
        ++iou_matches;
      }
    }
    const bool detection_passed = test_case.is_background ? raw_detections == 0 :
      iou_matches >= kRequiredMatches;
    const double duration = std::chrono::duration<double>(
      frames.back().received_at - frames.front().received_at).count();
    const double detection_rate = duration > 0.0 ? (frames.size() - 1) / duration : 0.0;
    const double inference_p95 = percentile95(inference);
    std::vector<std::string> failures;
    if (!detection_passed) {
      failures.push_back(test_case.is_background ?
        "raw-frame false positive detected" : "fewer than 7 IoU>=0.50 matches in 15 frames");
    }
    if (detection_rate < kMinimumDetectionRateHz) {
      failures.emplace_back("detection rate below 5 Hz");
    }
    if (inference_p95 > kMaximumInferenceP95Ms) {
      failures.emplace_back("inference P95 above 200 ms");
    }
    const bool passed = failures.empty();
    const std::string overlay = save_overlay(test_case, debug_image, passed, failures);
    json raw_counts = json::object();
    for (const auto & [name, count] : raw_class_frames) {
      raw_counts[name] = count;
    }
    return {
      {"case_index", test_case.index}, {"case_id", test_case.id}, {"world", test_case.world},
      {"mode", test_case.mode}, {"passed", passed}, {"detection_passed", detection_passed},
      {"expected_class", expected.empty() ? "none" : expected}, {"distance_m", test_case.distance},
      {"iou_threshold", test_case.is_background ? json(nullptr) : json(0.5)},
      {"ground_truth_box", truth ? json({{"x", truth->x}, {"y", truth->y},
          {"width", truth->width}, {"height", truth->height}}) : json(nullptr)},
      {"best_iou", truth ? json(best_iou) : json(nullptr)}, {"iou_matches", iou_matches},
      {"raw_frame_false_positive_count", test_case.is_background ? raw_detections : 0},
      {"raw_class_detections", raw_counts}, {"detection_rate_hz", detection_rate},
      {"inference_p95_ms", inference_p95}, {"overlay", overlay}, {"failures", failures}};
  }

  json failed_case_report(const CaseSpec & test_case, const std::string & reason) const
  {
    return {
      {"case_index", test_case.index}, {"case_id", test_case.id}, {"world", test_case.world},
      {"mode", test_case.mode}, {"passed", false}, {"detection_passed", false},
      {"expected_class", test_case.primary ? test_case.primary->target_class : "none"},
      {"distance_m", test_case.distance}, {"iou_threshold", test_case.primary ? json(0.5) : json(nullptr)},
      {"ground_truth_box", json(nullptr)}, {"best_iou", json(nullptr)}, {"iou_matches", 0},
      {"raw_frame_false_positive_count", 0}, {"raw_class_detections", json::object()},
      {"detection_rate_hz", 0.0}, {"inference_p95_ms", json(nullptr)}, {"overlay", "none"},
      {"failures", json::array({reason})}};
  }

  std::string save_overlay(
    const CaseSpec & test_case, const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const bool passed, const std::vector<std::string> & failures) const
  {
    if (!image) {
      return "none";
    }
    auto overlay = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
    cv::putText(
      overlay, test_case.id + (passed ? " PASS" : " FAIL"), cv::Point(12, 24),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, passed ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
    if (!failures.empty()) {
      cv::putText(
        overlay, failures.front(), cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.40,
        cv::Scalar(0, 0, 255), 1);
    }
    const fs::path path = output_dir_ / "overlays" /
      ("case_" + std::to_string(test_case.index) + ".jpg");
    if (!cv::imwrite(path.string(), overlay)) {
      throw std::runtime_error("failed to write acceptance overlay");
    }
    return fs::relative(path, output_dir_).string();
  }

  bool write_report(const json & cases, const std::size_t expected_count) const
  {
    const std::size_t passed_count = static_cast<std::size_t>(std::count_if(
      cases.begin(), cases.end(), [](const json & item) {return item.at("passed").get<bool>();}));
    const bool scope_passed = cases.size() == expected_count && passed_count == expected_count;
    const json report = {
      {"schema_version", 1}, {"holdout_manifest", holdout_manifest_path_.string()},
      {"case_world", case_world_}, {"gazebo_world_name", world_name_},
      {"expected_case_count", expected_count},
      {"passed_case_count", passed_count}, {"scope_passed", scope_passed}, {"cases", cases}};
    std::ofstream report_stream(output_dir_ / "report.json");
    report_stream << report.dump(2) << '\n';
    if (!report_stream) {
      throw std::runtime_error("failed to write final perception JSON report");
    }
    std::ofstream markdown(output_dir_ / "report.md");
    markdown << "# 四类感知分世界验收报告\n\n"
             << "- case world: `" << case_world_ << "`\n"
             << "- Gazebo world: `" << world_name_ << "`\n"
             << "- scope: " << (scope_passed ? "PASS" : "FAIL") << " (" << passed_count << "/"
             << expected_count << ")\n\n"
             << "| case | expected | mode | result | IoU matches | raw FP | P95 ms |\n"
             << "| --- | --- | --- | --- | ---: | ---: | ---: |\n";
    for (const auto & item : cases) {
      markdown << "| " << item.at("case_id").get<std::string>() << " | "
               << item.at("expected_class").get<std::string>() << " | "
               << item.at("mode").get<std::string>() << " | "
               << (item.at("passed").get<bool>() ? "PASS" : "FAIL") << " | "
               << item.at("iou_matches") << " | "
               << item.at("raw_frame_false_positive_count") << " | ";
      if (item.at("inference_p95_ms").is_null()) {
        markdown << "-";
      } else {
        markdown << item.at("inference_p95_ms");
      }
      markdown << " |\n";
    }
    return scope_passed;
  }

  void prepare_output()
  {
    if (fs::exists(output_dir_)) {
      throw std::runtime_error("output_dir already exists and will not be overwritten: " +
        output_dir_.string());
    }
    fs::create_directories(output_dir_ / "overlays");
  }

  fs::path output_dir_;
  fs::path holdout_manifest_path_;
  std::string world_name_;
  std::string case_world_;
  std::vector<std::string> target_classes_;
  std::string robot_name_;
  std::string label_topic_;
  bool preview_;
  int case_index_;
  int settle_frames_;
  double case_timeout_;
  std::vector<std::string> spawned_entities_;

  mutable gz::transport::Node gz_node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inference_sub_;
  std::mutex data_mutex_;
  std::condition_variable data_condition_;
  std::uint64_t detection_sequence_{0};
  std::uint64_t inference_sequence_{0};
  std::uint64_t label_sequence_{0};
  bool collecting_{false};
  std::vector<FrameRecord> frames_;
  std::vector<float> inference_ms_;
  LabelFrame labels_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_debug_image_;
};

}  // namespace turtlebot3_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<turtlebot3_gazebo::FinalPerceptionAcceptance>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});
    std::exception_ptr run_error;
    try {
      node->run();
    } catch (...) {
      run_error = std::current_exception();
    }
    executor.cancel();
    spin_thread.join();
    if (run_error) {
      std::rethrow_exception(run_error);
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("final_perception_acceptance"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
