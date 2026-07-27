// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <cv_bridge/cv_bridge.hpp>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
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

constexpr double kAcceptanceTestX = -2.0;
constexpr double kAcceptanceTestY = 0.5;
constexpr double kMinimumTestPoseDistance = 1.0;

std::vector<std::array<double, 2>> parse_positions(const std::vector<double> & values)
{
  if (values.empty() || values.size() % 2 != 0) {
    throw std::invalid_argument("capture positions must contain x/y pairs");
  }
  std::vector<std::array<double, 2>> positions;
  for (std::size_t index = 0; index < values.size(); index += 2) {
    const double distance = std::hypot(
      values[index] - kAcceptanceTestX, values[index + 1] - kAcceptanceTestY);
    if (distance < kMinimumTestPoseDistance) {
      throw std::invalid_argument("capture position leaks the fixed acceptance test pose");
    }
    positions.push_back({values[index], values[index + 1]});
  }
  return positions;
}

std::uint64_t image_hash(const cv::Mat & image)
{
  const cv::Mat contiguous = image.isContinuous() ? image : image.clone();
  const auto * data = contiguous.ptr<std::uint8_t>();
  const std::size_t bytes = contiguous.total() * contiguous.elemSize();
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::size_t index = 0; index < bytes; ++index) {
    hash ^= data[index];
    hash *= 1099511628211ULL;
  }
  return hash;
}

class HouseHardNegativeGenerator : public rclcpp::Node
{
public:
  HouseHardNegativeGenerator()
  : Node("house_hard_negative_generator")
  {
    const auto output = declare_parameter<std::string>("output_dir", "");
    if (output.empty()) {
      throw std::invalid_argument("output_dir is required");
    }
    output_dir_ = fs::absolute(output);
    world_name_ = declare_parameter<std::string>("world_name", "default");
    robot_name_ = declare_parameter<std::string>("robot_name", "waffle_pi_cam");
    train_count_ = declare_parameter("train_count", 600);
    validation_count_ = declare_parameter("validation_count", 100);
    seed_ = declare_parameter<std::int64_t>("seed", 20260723);
    settle_frames_ = declare_parameter("settle_frames", 5);
    capture_timeout_ = declare_parameter("capture_timeout", 20.0);
    maximum_jitter_ = declare_parameter("maximum_jitter", 0.15);
    train_positions_ = parse_positions(declare_parameter<std::vector<double>>(
      "train_positions", {
        1.083, 1.090, 0.855, 4.872, -1.866, 4.661, -4.151, 4.392,
        -4.316, 0.217, -4.228, -2.601, -4.922, -1.308, -3.236, 3.217}));
    validation_positions_ = parse_positions(declare_parameter<std::vector<double>>(
      "validation_positions", {-0.691, 2.877, 0.004, 0.989}));
    if (train_count_ <= 0 || validation_count_ < 0 || settle_frames_ <= 0 ||
      capture_timeout_ <= 0.0 || maximum_jitter_ < 0.0 || maximum_jitter_ >= 0.5)
    {
      throw std::invalid_argument("hard-negative generator parameters are invalid");
    }

    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/camera/color/image_raw");
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&HouseHardNegativeGenerator::on_image, this, std::placeholders::_1));
  }

  void run()
  {
    prepare_output();
    wait_for_infrastructure();
    std::ofstream metadata(output_dir_ / "metadata.jsonl");
    if (!metadata) {
      throw std::runtime_error("cannot create hard-negative metadata file");
    }

    const int target_count = train_count_ + validation_count_;
    int accepted = 0;
    int attempts = 0;
    double minimum_test_distance = std::numeric_limits<double>::infinity();
    while (accepted < target_count && attempts < target_count * 10 && rclcpp::ok()) {
      const bool is_train = accepted < train_count_;
      const std::string split = is_train ? "train" : "val";
      const int split_index = is_train ? accepted : accepted - train_count_;
      const auto & positions = is_train ? train_positions_ : validation_positions_;
      const std::uint64_t split_seed = static_cast<std::uint64_t>(seed_) +
        (is_train ? 0ULL : 1000000007ULL);
      const CapturePose pose = make_hard_negative_pose(
        static_cast<std::size_t>(attempts), split_seed, positions, maximum_jitter_);
      ++attempts;
      const double test_distance = std::hypot(
        pose.x - kAcceptanceTestX, pose.y - kAcceptanceTestY);
      if (test_distance < kMinimumTestPoseDistance) {
        continue;
      }

      set_robot_pose(pose);
      const auto message = wait_for_settled_image();
      cv::Mat image = cv_bridge::toCvCopy(
        message, sensor_msgs::image_encodings::BGR8)->image;
      if (image.cols != 640 || image.rows != 480) {
        throw std::runtime_error("hard-negative RGB image must be 640x480");
      }
      const auto mean = cv::mean(image);
      if (mean[0] + mean[1] + mean[2] < 6.0 || !image_hashes_.insert(image_hash(image)).second) {
        continue;
      }

      std::ostringstream name;
      name << "house_hn_" << split << "_" << std::setw(6) << std::setfill('0') << split_index;
      const fs::path image_path = output_dir_ / "images" / split / (name.str() + ".jpg");
      const fs::path label_path = output_dir_ / "labels" / split / (name.str() + ".txt");
      if (!cv::imwrite(image_path.string(), image)) {
        throw std::runtime_error("failed to write hard-negative image");
      }
      std::ofstream label(label_path);
      if (!label) {
        throw std::runtime_error("failed to write empty hard-negative label");
      }
      metadata << json({
        {"name", name.str()}, {"split", split}, {"seed", split_seed},
        {"attempt", attempts}, {"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw},
        {"acceptance_test_pose_distance_m", test_distance}}).dump() << '\n';
      minimum_test_distance = std::min(minimum_test_distance, test_distance);
      ++accepted;
      if (accepted % 50 == 0 || accepted == target_count) {
        RCLCPP_INFO(get_logger(), "Captured %d/%d hard negatives", accepted, target_count);
      }
    }
    if (accepted != target_count) {
      throw std::runtime_error("could not obtain the requested number of unique hard negatives");
    }
    metadata.close();
    write_report(attempts, minimum_test_distance);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = message;
    ++image_sequence_;
    image_condition_.notify_all();
  }

  std::string service(const std::string & name) const
  {
    return "/world/" + world_name_ + "/" + name;
  }

  void prepare_output() const
  {
    if (fs::exists(output_dir_) && (!fs::is_directory(output_dir_) || !fs::is_empty(output_dir_))) {
      throw std::runtime_error("hard-negative output directory must be new or empty");
    }
    for (const auto & split : {"train", "val"}) {
      fs::create_directories(output_dir_ / "images" / split);
      fs::create_directories(output_dir_ / "labels" / split);
    }
  }

  void wait_for_infrastructure()
  {
    bool service_ready = false;
    for (int attempt = 0; attempt < 60 && rclcpp::ok(); ++attempt) {
      std::vector<std::string> services;
      gz_node_.ServiceList(services);
      service_ready = std::find(services.begin(), services.end(), service("set_pose")) !=
        services.end();
      if (service_ready) {
        break;
      }
      std::this_thread::sleep_for(500ms);
    }
    if (!service_ready) {
      throw std::runtime_error("Gazebo set_pose service is unavailable");
    }
    std::unique_lock<std::mutex> lock(image_mutex_);
    if (!image_condition_.wait_for(lock, 30s, [this]() {return latest_image_ != nullptr;})) {
      throw std::runtime_error("RGB camera image is unavailable");
    }
  }

  void set_robot_pose(const CapturePose & pose) const
  {
    gz::msgs::Pose request;
    request.set_name(robot_name_);
    request.mutable_position()->set_x(pose.x);
    request.mutable_position()->set_y(pose.y);
    request.mutable_position()->set_z(0.01);
    request.mutable_orientation()->set_z(std::sin(pose.yaw / 2.0));
    request.mutable_orientation()->set_w(std::cos(pose.yaw / 2.0));
    gz::msgs::Boolean response;
    bool result = false;
    const bool executed = gz_node_.Request(service("set_pose"), request, 10000, response, result);
    if (!executed || !result || !response.data()) {
      throw std::runtime_error("Gazebo set_pose request failed");
    }
  }

  sensor_msgs::msg::Image::ConstSharedPtr wait_for_settled_image()
  {
    std::unique_lock<std::mutex> lock(image_mutex_);
    const std::uint64_t target = image_sequence_ + static_cast<std::uint64_t>(settle_frames_);
    const bool ready = image_condition_.wait_for(
      lock, std::chrono::duration<double>(capture_timeout_),
      [this, target]() {return image_sequence_ >= target;});
    if (!ready) {
      throw std::runtime_error("timed out waiting for a settled RGB frame");
    }
    return latest_image_;
  }

  void write_report(const int attempts, const double minimum_test_distance) const
  {
    const json report = {
      {"passed", true}, {"source_world", "turtlebot3_house"}, {"seed", seed_},
      {"train_images", train_count_}, {"validation_images", validation_count_},
      {"empty_labels", train_count_ + validation_count_}, {"attempts", attempts},
      {"unique_images", image_hashes_.size()}, {"image_size", {640, 480}},
      {"acceptance_test_pose", {kAcceptanceTestX, kAcceptanceTestY}},
      {"minimum_test_pose_distance_m", minimum_test_distance},
      {"minimum_required_test_pose_distance_m", kMinimumTestPoseDistance}};
    std::ofstream json_stream(output_dir_ / "report.json");
    json_stream << report.dump(2) << '\n';
    std::ofstream markdown(output_dir_ / "report.md");
    markdown << "# TurtleBot3 House Hard-Negative Dataset\n\n"
             << "- Result: PASS\n"
             << "- Train images: " << train_count_ << "\n"
             << "- Validation images: " << validation_count_ << "\n"
             << "- Empty labels: " << train_count_ + validation_count_ << "\n"
             << "- Unique images: " << image_hashes_.size() << "\n"
             << "- Minimum distance from acceptance pose: " << minimum_test_distance << " m\n";
  }

  fs::path output_dir_;
  std::string world_name_;
  std::string robot_name_;
  int train_count_;
  int validation_count_;
  std::int64_t seed_;
  int settle_frames_;
  double capture_timeout_;
  double maximum_jitter_;
  std::vector<std::array<double, 2>> train_positions_;
  std::vector<std::array<double, 2>> validation_positions_;
  mutable gz::transport::Node gz_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::mutex image_mutex_;
  std::condition_variable image_condition_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::uint64_t image_sequence_{0};
  std::set<std::uint64_t> image_hashes_;
};

}  // namespace turtlebot3_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<turtlebot3_gazebo::HouseHardNegativeGenerator>();
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
    RCLCPP_FATAL(rclcpp::get_logger("house_hard_negative_generator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
