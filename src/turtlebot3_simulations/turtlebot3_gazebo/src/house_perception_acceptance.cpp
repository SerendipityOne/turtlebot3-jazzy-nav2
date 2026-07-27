// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
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
constexpr double kMaximumPositionError = 0.30;
constexpr double kMinimumDetectionRate = 5.0;
constexpr double kMaximumInferenceP95Ms = 200.0;
constexpr double kMaximumDetectionAgeMs = 500.0;
constexpr std::size_t kMaximumConsecutiveStaleFrames = 2;
constexpr double kRobotX = -2.0;
constexpr double kRobotY = 0.5;
constexpr double kRobotZ = 0.01;
constexpr double kCameraForwardOffset = 0.08;
constexpr double kPlatformTopZ = 0.22;
constexpr double kPi = 3.14159265358979323846;
const std::array<double, 3> kDistances{0.75, 1.50, 2.50};
const std::vector<std::string> kEnabledClasses{"cup", "backpack", "ball", "box"};

struct CaseSpec
{
  std::size_t index;
  const DatasetAsset * asset;
  double distance;
  double robot_yaw;
  bool is_negative;
};

struct FrameRecord
{
  vision_msgs::msg::Detection3DArray::ConstSharedPtr detections;
  std::chrono::steady_clock::time_point received_at;
  double age_ms;
};

std::string detection_class(const std::string & label)
{
  return label.substr(0, label.find(':'));
}

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

class HousePerceptionAcceptance : public rclcpp::Node
{
public:
  HousePerceptionAcceptance()
  : Node("house_perception_acceptance"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    output_dir_ = fs::absolute(declare_parameter<std::string>("output_dir"));
    manifest_path_ = fs::absolute(declare_parameter<std::string>("asset_manifest"));
    world_name_ = declare_parameter<std::string>("world_name", "default");
    robot_name_ = declare_parameter<std::string>("robot_name", "waffle_pi_cam");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    preview_ = declare_parameter("preview", false);
    case_index_ = declare_parameter("case_index", -1);
    settle_frames_ = declare_parameter("settle_frames", 10);
    case_timeout_ = declare_parameter("case_timeout", 30.0);

    if (output_dir_.empty() || manifest_path_.empty() || settle_frames_ <= 0 ||
      case_timeout_ <= 0.0 || case_index_ < -1 || case_index_ >= 20)
    {
      throw std::invalid_argument("invalid house acceptance parameters");
    }
    detections_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "/embodied/detections", 10,
      std::bind(&HousePerceptionAcceptance::on_detections, this, std::placeholders::_1));
    debug_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/embodied/debug_image", 10,
      std::bind(&HousePerceptionAcceptance::on_debug_image, this, std::placeholders::_1));
    inference_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/embodied/inference_ms", 10,
      std::bind(&HousePerceptionAcceptance::on_inference, this, std::placeholders::_1));
  }

  void run()
  {
    prepare_output();
    assets_ = load_asset_manifest(manifest_path_);
    select_test_assets();
    wait_for_infrastructure();
    const auto all_cases = build_cases();
    std::vector<CaseSpec> selected_cases;
    if (preview_ || case_index_ >= 0) {
      const int selected = case_index_ >= 0 ? case_index_ : 0;
      selected_cases.push_back(all_cases.at(static_cast<std::size_t>(selected)));
    } else {
      selected_cases = all_cases;
    }

    json case_reports = json::array();
    for (const auto & test_case : selected_cases) {
      // Scene-control failures indicate broken infrastructure and must stop the run immediately.
      configure_scene(test_case);
      try {
        case_reports.push_back(run_case(test_case));
      } catch (const std::exception & error) {
        // Detection/case failures are evidence; Gazebo service failures are thrown by configure_scene.
        RCLCPP_WARN(
          get_logger(), "Case %zu failed and will be recorded: %s", test_case.index, error.what());
        case_reports.push_back(failed_case_report(test_case, error.what()));
      }
    }
    remove_spawned_entities();
    const bool passed = write_report(case_reports, selected_cases.size());
    if (!passed) {
      throw std::runtime_error("house perception acceptance failed; see report.json");
    }
    RCLCPP_INFO(get_logger(), "House perception acceptance passed: %s", output_dir_.c_str());
  }

private:
  void on_detections(const vision_msgs::msg::Detection3DArray::ConstSharedPtr message)
  {
    double age_ms = -1.0;
    if (message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0) {
      age_ms = (now() - rclcpp::Time(message->header.stamp)).seconds() * 1000.0;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++detection_sequence_;
    latest_frame_id_ = message->header.frame_id;
    if (collecting_) {
      frames_.push_back(FrameRecord{message, std::chrono::steady_clock::now(), age_ms});
    }
    data_condition_.notify_all();
  }

  void on_debug_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_debug_image_ = message;
    data_condition_.notify_all();
  }

  void on_inference(const std_msgs::msg::Float32::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    inference_seen_ = true;
    if (collecting_) {
      inference_ms_.push_back(message->data);
    }
    data_condition_.notify_all();
  }

  void prepare_output() const
  {
    if (fs::exists(output_dir_ / "report.json")) {
      throw std::runtime_error("output directory already contains report.json");
    }
    fs::create_directories(output_dir_ / "overlays");
  }

  void select_test_assets()
  {
    for (const auto & asset : assets_) {
      if (asset.split == "test" &&
        std::find(kEnabledClasses.begin(), kEnabledClasses.end(), asset.target_class) !=
        kEnabledClasses.end())
      {
        if (!fs::is_regular_file(asset.sdf_path)) {
          throw std::runtime_error("missing test asset: " + asset.sdf_path.string());
        }
        test_assets_.push_back(&asset);
      }
    }
    if (test_assets_.size() != 5) {
      throw std::runtime_error("expected exactly five non-bottle test assets");
    }
  }

  std::string service(const std::string & name) const
  {
    return "/world/" + world_name_ + "/" + name;
  }

  void wait_for_infrastructure()
  {
    for (int attempt = 0; attempt < 60 && rclcpp::ok(); ++attempt) {
      std::vector<std::string> services;
      gz_node_.ServiceList(services);
      if (std::find(services.begin(), services.end(), service("create")) != services.end() &&
        std::find(services.begin(), services.end(), service("set_pose")) != services.end() &&
        std::find(services.begin(), services.end(), service("remove")) != services.end())
      {
        break;
      }
      if (attempt == 59) {
        throw std::runtime_error("Gazebo user command services are unavailable");
      }
      std::this_thread::sleep_for(500ms);
    }

    std::unique_lock<std::mutex> lock(data_mutex_);
    const bool topics_ready = data_condition_.wait_for(lock, 30s, [&]() {
        return detection_sequence_ > 0 && latest_debug_image_ && inference_seen_ &&
               !latest_frame_id_.empty();
      });
    const std::string camera_frame = latest_frame_id_;
    lock.unlock();
    if (!topics_ready) {
      throw std::runtime_error("camera, detector, debug image or inference metric is unavailable");
    }
    if (!tf_buffer_.canTransform(base_frame_, camera_frame, tf2::TimePointZero, 10s)) {
      throw std::runtime_error("camera-to-base TF is unavailable: " + camera_frame);
    }
  }

  template<typename Request>
  void request_boolean(const std::string & service_name, const Request & request) const
  {
    gz::msgs::Boolean response;
    bool result = false;
    const bool executed = gz_node_.Request(service_name, request, 10000, response, result);
    if (!executed || !result || !response.data()) {
      throw std::runtime_error("Gazebo service failed: " + service_name);
    }
  }

  void set_robot_pose(const double yaw) const
  {
    gz::msgs::Pose request;
    request.set_name(robot_name_);
    request.mutable_position()->set_x(kRobotX);
    request.mutable_position()->set_y(kRobotY);
    request.mutable_position()->set_z(kRobotZ);
    request.mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_orientation()->set_w(std::cos(yaw / 2.0));
    request_boolean(service("set_pose"), request);
  }

  void spawn_file(
    const std::string & name, const fs::path & path, const double x, const double y,
    const double z, const double yaw)
  {
    gz::msgs::EntityFactory request;
    request.set_name(name);
    request.set_allow_renaming(false);
    request.set_sdf_filename(path.string());
    request.mutable_pose()->mutable_position()->set_x(x);
    request.mutable_pose()->mutable_position()->set_y(y);
    request.mutable_pose()->mutable_position()->set_z(z);
    request.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2.0));
    request_boolean(service("create"), request);
    spawned_entities_.push_back(name);
  }

  void spawn_platform(const std::string & name, const double x, const double y)
  {
    const std::string sdf =
      "<sdf version='1.8'><model name='acceptance_platform'><static>true</static>"
      "<link name='link'><collision name='collision'><geometry><box><size>0.40 0.40 0.22"
      "</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.40 "
      "0.40 0.22</size></box></geometry><material><diffuse>0.55 0.55 0.55 1</diffuse>"
      "</material></visual></link></model></sdf>";
    gz::msgs::EntityFactory request;
    request.set_name(name);
    request.set_allow_renaming(false);
    request.set_sdf(sdf);
    request.mutable_pose()->mutable_position()->set_x(x);
    request.mutable_pose()->mutable_position()->set_y(y);
    request.mutable_pose()->mutable_position()->set_z(kPlatformTopZ / 2.0);
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

  std::vector<CaseSpec> build_cases() const
  {
    std::vector<CaseSpec> cases;
    for (const auto * asset : test_assets_) {
      for (const double distance : kDistances) {
        cases.push_back(CaseSpec{cases.size(), asset, distance, 0.0, false});
      }
    }
    const std::array<double, 5> negative_yaws{0.0, kPi / 2.0, kPi, -kPi / 2.0, kPi / 4.0};
    for (const double yaw : negative_yaws) {
      cases.push_back(CaseSpec{cases.size(), nullptr, 0.0, yaw, true});
    }
    return cases;
  }

  std::array<double, 3> target_position(const CaseSpec & test_case) const
  {
    const double forward = kCameraForwardOffset + test_case.distance;
    const double x = kRobotX + std::cos(test_case.robot_yaw) * forward;
    const double y = kRobotY + std::sin(test_case.robot_yaw) * forward;
    const double z = test_case.asset->surface == "raised" ? kPlatformTopZ : 0.02;
    return {x, y, z + test_case.asset->z_offset};
  }

  void configure_scene(const CaseSpec & test_case)
  {
    remove_spawned_entities();
    set_robot_pose(test_case.robot_yaw);
    if (test_case.is_negative) {
      return;
    }
    const auto target = target_position(test_case);
    const std::string suffix = std::to_string(test_case.index);
    if (test_case.asset->surface == "raised") {
      spawn_platform("acceptance_platform_" + suffix, target[0], target[1]);
    }
    // The target's local front faces the camera; original meshes and textures remain unchanged.
    spawn_file(
      "acceptance_target_" + suffix, test_case.asset->sdf_path,
      target[0], target[1], target[2], test_case.robot_yaw + kPi);
  }

  void collect_case_frames(
    std::vector<FrameRecord> & frames, std::vector<float> & inference,
    sensor_msgs::msg::Image::ConstSharedPtr & debug_image)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const auto settle_target = detection_sequence_ + static_cast<std::uint64_t>(settle_frames_);
    if (!data_condition_.wait_for(lock, std::chrono::duration<double>(case_timeout_), [&]() {
        return detection_sequence_ >= settle_target;
      }))
    {
      throw std::runtime_error("timed out while settling detector frames");
    }
    frames_.clear();
    inference_ms_.clear();
    collecting_ = true;
    const bool collected = data_condition_.wait_for(
      lock, std::chrono::duration<double>(case_timeout_), [&]() {
        return frames_.size() >= kWindowSize && inference_ms_.size() >= kWindowSize;
      });
    collecting_ = false;
    if (!collected) {
      throw std::runtime_error("timed out collecting the 15-frame acceptance window");
    }
    frames.assign(frames_.begin(), frames_.begin() + kWindowSize);
    inference.assign(inference_ms_.begin(), inference_ms_.begin() + kWindowSize);
    debug_image = latest_debug_image_;
  }

  json run_case(const CaseSpec & test_case)
  {
    std::vector<FrameRecord> records;
    std::vector<float> inference;
    sensor_msgs::msg::Image::ConstSharedPtr debug_image;
    collect_case_frames(records, inference, debug_image);

    std::vector<AcceptanceFrame> acceptance_frames;
    std::map<std::string, std::size_t> class_frame_counts;
    double best_error = std::numeric_limits<double>::infinity();
    std::array<double, 3> best_world{
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
    const auto truth = test_case.is_negative ? std::array<double, 3>{0.0, 0.0, 0.0} :
      target_position(test_case);

    for (const auto & record : records) {
      AcceptanceFrame frame;
      std::set<std::string> unique_classes;
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, record.detections->header.frame_id, tf2::TimePointZero, 1s);
      for (const auto & detection : record.detections->detections) {
        const std::string name = detection_class(detection.id);
        frame.detected_classes.push_back(name);
        unique_classes.insert(name);
        if (test_case.is_negative || name != test_case.asset->target_class) {
          continue;
        }
        geometry_msgs::msg::PointStamped camera_point;
        camera_point.header = record.detections->header;
        camera_point.point = detection.bbox.center.position;
        geometry_msgs::msg::PointStamped base_point;
        tf2::doTransform(camera_point, base_point, transform);
        if (!std::isfinite(base_point.point.x) || !std::isfinite(base_point.point.y) ||
          !std::isfinite(base_point.point.z) || base_point.point.x <= 0.0)
        {
          continue;
        }
        const double world_x = kRobotX +
          std::cos(test_case.robot_yaw) * base_point.point.x -
          std::sin(test_case.robot_yaw) * base_point.point.y;
        const double world_y = kRobotY +
          std::sin(test_case.robot_yaw) * base_point.point.x +
          std::cos(test_case.robot_yaw) * base_point.point.y;
        const double error = std::hypot(world_x - truth[0], world_y - truth[1]);
        if (error < best_error) {
          best_error = error;
          best_world = {world_x, world_y, kRobotZ + base_point.point.z};
        }
        if (error <= kMaximumPositionError) {
          frame.has_valid_expected_location = true;
        }
      }
      for (const auto & name : unique_classes) {
        ++class_frame_counts[name];
      }
      acceptance_frames.push_back(std::move(frame));
    }

    const std::string expected = test_case.is_negative ? "" : test_case.asset->target_class;
    const auto decision = evaluate_acceptance_case(
      acceptance_frames, expected, kEnabledClasses, kRequiredMatches, test_case.is_negative);
    const double duration = std::chrono::duration<double>(
      records.back().received_at - records.front().received_at).count();
    const double detection_rate = duration > 0.0 ? (records.size() - 1) / duration : 0.0;
    const double inference_p95 = percentile95(inference);
    double maximum_age = 0.0;
    std::size_t stale_run = 0;
    std::size_t maximum_stale_run = 0;
    for (const auto & record : records) {
      maximum_age = std::max(maximum_age, record.age_ms);
      stale_run = record.age_ms > kMaximumDetectionAgeMs ? stale_run + 1 : 0;
      maximum_stale_run = std::max(maximum_stale_run, stale_run);
    }

    std::vector<std::string> failures;
    if (!decision.passed) {
      failures.emplace_back("7-of-15 class/location gate failed");
    }
    if (detection_rate < kMinimumDetectionRate) {
      failures.emplace_back("detection rate below 5 Hz");
    }
    if (inference_p95 > kMaximumInferenceP95Ms) {
      failures.emplace_back("inference P95 above 200 ms");
    }
    if (maximum_stale_run > kMaximumConsecutiveStaleFrames) {
      failures.emplace_back("sustained stale image processing");
    }
    const bool passed = failures.empty();
    const std::string overlay = save_overlay(test_case, debug_image, passed, failures);

    json class_counts = json::object();
    for (const auto & [name, count] : class_frame_counts) {
      class_counts[name] = count;
    }
    return {
      {"case_index", test_case.index}, {"passed", passed},
      {"negative", test_case.is_negative}, {"asset_id", test_case.is_negative ? "none" : test_case.asset->id},
      {"expected_class", test_case.is_negative ? "none" : expected},
      {"distance_m", test_case.distance}, {"robot_yaw", test_case.robot_yaw},
      {"truth_xyz", test_case.is_negative ? json(nullptr) : json(truth)},
      {"best_detection_world_xyz", std::isfinite(best_error) ? json(best_world) : json(nullptr)},
      {"best_xy_error_m", std::isfinite(best_error) ? json(best_error) : json(nullptr)},
      {"expected_valid_matches", decision.expected_location_matches},
      {"bottle_frames", decision.bottle_frames},
      {"class_frame_counts", class_counts}, {"confirmed_classes", decision.confirmed_classes},
      {"detection_rate_hz", detection_rate}, {"inference_p95_ms", inference_p95},
      {"maximum_detection_age_ms", maximum_age},
      {"maximum_consecutive_stale_frames", maximum_stale_run},
      {"overlay", overlay}, {"failures", failures}};
  }

  json failed_case_report(const CaseSpec & test_case, const std::string & reason) const
  {
    return {
      {"case_index", test_case.index}, {"passed", false}, {"negative", test_case.is_negative},
      {"asset_id", test_case.is_negative ? "none" : test_case.asset->id},
      {"expected_class", test_case.is_negative ? "none" : test_case.asset->target_class},
      {"distance_m", test_case.distance}, {"failures", json::array({reason})}};
  }

  std::string save_overlay(
    const CaseSpec & test_case, const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const bool passed, const std::vector<std::string> & failures) const
  {
    if (!image) {
      return "none";
    }
    cv::Mat overlay = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
    const std::string summary = "case " + std::to_string(test_case.index) +
      (passed ? " PASS" : " FAIL");
    cv::putText(
      overlay, summary, cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.65,
      passed ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
    if (!failures.empty()) {
      cv::putText(
        overlay, failures.front(), cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.45,
        cv::Scalar(0, 0, 255), 1);
    }
    std::ostringstream name;
    name << "case_" << std::setw(2) << std::setfill('0') << test_case.index << ".jpg";
    const fs::path path = output_dir_ / "overlays" / name.str();
    if (!cv::imwrite(path.string(), overlay)) {
      throw std::runtime_error("failed to write case overlay");
    }
    return fs::relative(path, output_dir_).string();
  }

  bool write_report(const json & cases, const std::size_t expected_count) const
  {
    const std::size_t passed_count = static_cast<std::size_t>(std::count_if(
      cases.begin(), cases.end(), [](const json & item) {return item.at("passed").get<bool>();}));
    const bool passed = cases.size() == expected_count && passed_count == expected_count;
    const json report = {
      {"passed", passed}, {"mode", preview_ ? "preview" : "full"},
      {"passed_cases", passed_count}, {"total_cases", cases.size()},
      {"asset_manifest", manifest_path_.string()},
      {"locked_gates", {
          {"enabled_classes", kEnabledClasses}, {"disabled_class_ids", {1}},
          {"window_size", kWindowSize}, {"required_matches", kRequiredMatches},
          {"maximum_xy_error_m", kMaximumPositionError},
          {"minimum_detection_rate_hz", kMinimumDetectionRate},
          {"maximum_inference_p95_ms", kMaximumInferenceP95Ms},
          {"maximum_detection_age_ms", kMaximumDetectionAgeMs},
          {"maximum_consecutive_stale_frames", kMaximumConsecutiveStaleFrames},
          {"confidence_threshold", 0.261}, {"nms_threshold", 0.45}}},
      {"cases", cases}};
    std::ofstream(output_dir_ / "report.json") << report.dump(2) << '\n';

    std::ofstream markdown(output_dir_ / "report.md");
    markdown << "# TurtleBot3 House 静态感知验收报告\n\n"
             << "- 结果：" << (passed ? "PASS" : "FAIL") << "\n"
             << "- 通过案例：" << passed_count << "/" << cases.size() << "\n"
             << "- 多帧规则：最近 15 帧匹配 7 帧\n"
             << "- bottle：运行时禁用\n\n"
             << "| Case | 类别 | 距离(m) | 结果 | 有效匹配 | XY误差(m) | Hz | P95(ms) |\n"
             << "|---:|---|---:|---|---:|---:|---:|---:|\n";
    for (const auto & item : cases) {
      markdown << "| " << item.at("case_index") << " | "
               << item.at("expected_class").get<std::string>()
               << " | " << item.value("distance_m", 0.0) << " | "
               << (item.at("passed").get<bool>() ? "PASS" : "FAIL") << " | "
               << item.value("expected_valid_matches", 0) << " | ";
      if (item.contains("best_xy_error_m") && !item.at("best_xy_error_m").is_null()) {
        markdown << item.at("best_xy_error_m");
      } else {
        markdown << "-";
      }
      markdown << " | " << item.value("detection_rate_hz", 0.0)
               << " | " << item.value("inference_p95_ms", 0.0) << " |\n";
    }
    return passed;
  }

  fs::path output_dir_;
  fs::path manifest_path_;
  std::string world_name_;
  std::string robot_name_;
  std::string base_frame_;
  bool preview_;
  int case_index_;
  int settle_frames_;
  double case_timeout_;
  std::vector<DatasetAsset> assets_;
  std::vector<const DatasetAsset *> test_assets_;
  std::vector<std::string> spawned_entities_;

  mutable gz::transport::Node gz_node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inference_sub_;
  std::mutex data_mutex_;
  std::condition_variable data_condition_;
  std::uint64_t detection_sequence_{0};
  std::string latest_frame_id_;
  bool inference_seen_{false};
  bool collecting_{false};
  std::vector<FrameRecord> frames_;
  std::vector<float> inference_ms_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_debug_image_;
};

}  // namespace turtlebot3_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<turtlebot3_gazebo::HousePerceptionAcceptance>();
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
    RCLCPP_FATAL(rclcpp::get_logger("house_perception_acceptance"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
