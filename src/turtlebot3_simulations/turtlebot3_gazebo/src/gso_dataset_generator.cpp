#include <cv_bridge/cv_bridge.hpp>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "turtlebot3_gazebo/dataset_utils.hpp"

namespace turtlebot3_gazebo
{

namespace fs = std::filesystem;
using Image = sensor_msgs::msg::Image;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
using json = nlohmann::json;
using namespace std::chrono_literals;

constexpr std::array<const char *, 5> kClassNames = {
  "cup", "bottle", "backpack", "ball", "box"};

using Asset = DatasetAsset;

struct Task
{
  std::string split;
  int primary_class_id;
  bool is_negative;
  bool has_randomized_background;
  std::size_t primary_asset_ordinal;
};

struct Scene
{
  std::uint64_t seed;
  std::string background_mode;
  int zone;
  double robot_x;
  double robot_y;
  double robot_yaw;
  std::vector<const Asset *> active_assets;
  int primary_class_id;
};

struct Frame
{
  Image::ConstSharedPtr color;
  Image::ConstSharedPtr depth;
  std::vector<std::uint8_t> labels;
  std::uint32_t label_width{0};
  std::uint32_t label_height{0};
  std::uint32_t label_step{0};
  std::int64_t color_stamp{0};
  std::int64_t label_stamp{0};
};

std::int64_t ros_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

std::int64_t gz_stamp(const gz::msgs::Header & header)
{
  if (!header.has_stamp()) {
    return 0;
  }
  return static_cast<std::int64_t>(header.stamp().sec()) * 1000000000LL +
         header.stamp().nsec();
}

class GsoDatasetGenerator : public rclcpp::Node
{
public:
  GsoDatasetGenerator()
  : Node("gso_dataset_generator"),
    color_sub_(this, declare_parameter("color_topic", "/camera/color/image_raw"),
      rmw_qos_profile_sensor_data),
    depth_sub_(this, declare_parameter("depth_topic", "/camera/depth/image_raw"),
      rmw_qos_profile_sensor_data),
    sync_(SyncPolicy(12), color_sub_, depth_sub_)
  {
    output_dir_ = fs::absolute(declare_parameter<std::string>("output_dir"));
    manifest_path_ = fs::absolute(declare_parameter<std::string>("asset_manifest"));
    approval_file_ = fs::absolute(declare_parameter<std::string>("approval_file", "approval.txt"));
    target_count_ = declare_parameter("target_count", 100);
    master_seed_ = declare_parameter<std::int64_t>("seed", 20260720);
    settle_frames_ = declare_parameter("settle_frames", 15);
    capture_timeout_ = declare_parameter("capture_timeout", 45.0);
    raised_distance_min_ = declare_parameter("raised_distance_min", 1.4);
    raised_distance_max_ = declare_parameter("raised_distance_max", 1.7);
    floor_distance_min_ = declare_parameter("floor_distance_min", 0.8);
    floor_distance_max_ = declare_parameter("floor_distance_max", 2.5);
    minimum_visible_pixels_ = declare_parameter("minimum_visible_pixels", 256);
    minimum_box_dimension_ = declare_parameter("minimum_box_dimension", 16);
    maximum_box_area_ratio_ = declare_parameter("maximum_box_area_ratio", 0.75);
    maximum_consecutive_failures_ = declare_parameter("maximum_consecutive_failures", 20);
    require_full_primary_coverage_ = declare_parameter("require_full_primary_coverage", false);
    maximum_rejection_ratio_ = declare_parameter("maximum_rejection_ratio", -1.0);
    negative_only_ = declare_parameter("negative_only", false);
    negative_train_count_ = declare_parameter("negative_train_count", 0);
    negative_validation_count_ = declare_parameter("negative_validation_count", 0);
    robot_name_ = declare_parameter<std::string>("robot_name", "waffle_pi_cam");
    world_name_ = declare_parameter<std::string>("world_name", "gso_dataset");
    label_topic_ = declare_parameter<std::string>(
      "label_topic", "/dataset/segmentation/labels_map");

    if (target_count_ <= 0 || settle_frames_ <= 0 || capture_timeout_ <= 0.0 ||
      minimum_visible_pixels_ <= 0 || minimum_box_dimension_ <= 0 ||
      maximum_box_area_ratio_ <= 0.0 || maximum_box_area_ratio_ > 1.0 ||
      maximum_consecutive_failures_ <= 0 ||
      (maximum_rejection_ratio_ < 0.0 && maximum_rejection_ratio_ != -1.0) ||
      !std::isfinite(raised_distance_min_) || !std::isfinite(raised_distance_max_) ||
      !std::isfinite(floor_distance_min_) || !std::isfinite(floor_distance_max_) ||
      raised_distance_min_ <= 0.0 || raised_distance_max_ < raised_distance_min_ ||
      floor_distance_min_ <= 0.0 || floor_distance_max_ < floor_distance_min_ ||
      negative_train_count_ < 0 || negative_validation_count_ < 0 ||
      (negative_only_ && (negative_train_count_ <= 0 || negative_validation_count_ <= 0 ||
      target_count_ != negative_train_count_ + negative_validation_count_)) ||
      (!negative_only_ && (negative_train_count_ != 0 || negative_validation_count_ != 0)))
    {
      throw std::invalid_argument("dataset generator numeric parameters and distance ranges must be valid");
    }

    sync_.registerCallback(
      std::bind(&GsoDatasetGenerator::on_rgbd, this, std::placeholders::_1, std::placeholders::_2));
    if (!gz_node_.Subscribe(label_topic_, &GsoDatasetGenerator::on_labels, this)) {
      throw std::runtime_error("failed to subscribe to Gazebo label topic: " + label_topic_);
    }
    pose_topic_ = "/world/" + world_name_ + "/pose/info";
    if (!gz_node_.Subscribe(pose_topic_, &GsoDatasetGenerator::on_poses, this)) {
      throw std::runtime_error("failed to subscribe to Gazebo pose topic: " + pose_topic_);
    }
  }

  void run()
  {
    enforce_manual_gate();
    prepare_directories();
    assets_ = load_assets();
    validate_assets();
    wait_for_gazebo();

    auto tasks = build_tasks();
    auto state = load_state();
    int accepted = state.value("accepted", 0);
    int attempts = state.value("attempts", 0);
    int consecutive_failures = 0;
    const auto overlay_indices = build_overlay_indices(tasks);

    RCLCPP_INFO(get_logger(), "Generating %d samples from index %d", target_count_, accepted);
    while (accepted < target_count_ && rclcpp::ok()) {
      const Task & task = tasks.at(static_cast<std::size_t>(accepted));
      const auto sample_seed = static_cast<std::uint64_t>(master_seed_) * 1000003ULL + attempts;
      ++attempts;
      std::optional<Scene> scene;
      try {
        scene = configure_scene(task, sample_seed);
        const Frame frame = wait_for_frame(task);
        save_sample(
          accepted, task, *scene, frame,
          overlay_indices.count(static_cast<std::size_t>(accepted)) != 0);
        ++accepted;
        consecutive_failures = 0;
        write_state(accepted, attempts);
        if (accepted % 10 == 0 || accepted == target_count_) {
          RCLCPP_INFO(get_logger(), "Accepted %d/%d samples", accepted, target_count_);
        }
      } catch (const std::exception & error) {
        ++consecutive_failures;
        write_rejection(attempts, sample_seed, task, scene, error.what());
        write_state(accepted, attempts);
        RCLCPP_WARN(
          get_logger(), "Rejected attempt %d (%d consecutive): %s", attempts,
          consecutive_failures, error.what());
        if (consecutive_failures >= maximum_consecutive_failures_) {
          throw std::runtime_error("maximum consecutive failures reached");
        }
      }
    }
    if (!rclcpp::ok()) {
      throw std::runtime_error("generation interrupted before reaching target_count");
    }
    write_report();
    RCLCPP_INFO(get_logger(), "Dataset complete: %s", output_dir_.c_str());
  }

  void stop_transport()
  {
    // Gazebo callbacks carry this node pointer, so detach them before node destruction.
    for (const auto & topic : {label_topic_, pose_topic_}) {
      if (!gz_node_.Unsubscribe(topic)) {
        RCLCPP_WARN(get_logger(), "Gazebo topic was already unsubscribed: %s", topic.c_str());
      }
    }
  }

private:
  void on_rgbd(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth)
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_color_ = color;
    latest_depth_ = depth;
    ++rgbd_sequence_;
    frame_condition_.notify_all();
  }

  void on_labels(const gz::msgs::Image & message)
  {
    if (message.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Label map must use Gazebo RGB_INT8 format");
      return;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_labels_.assign(message.data().begin(), message.data().end());
    label_width_ = message.width();
    label_height_ = message.height();
    label_step_ = message.step();
    label_stamp_ = gz_stamp(message.header());
    ++label_sequence_;
    frame_condition_.notify_all();
  }

  void on_poses(const gz::msgs::Pose_V & message)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    known_entity_positions_.clear();
    for (const auto & pose : message.pose()) {
      known_entity_positions_[pose.name()] = {
        pose.position().x(), pose.position().y()};
    }
    pose_condition_.notify_all();
  }

  void enforce_manual_gate() const
  {
    if (target_count_ <= 100) {
      return;
    }
    std::ifstream approval(approval_file_);
    std::string line;
    std::getline(approval, line);
    const std::string expected = "approved_target_count=" + std::to_string(target_count_);
    if (!approval || line != expected) {
      throw std::runtime_error(
              "target_count > 100 requires approval file containing: " + expected);
    }
  }

  void prepare_directories() const
  {
    for (const auto * split : {"train", "val", "test"}) {
      for (const auto * directory : {"images", "labels", "depth", "instance", "metadata"}) {
        fs::create_directories(output_dir_ / directory / split);
      }
    }
    fs::create_directories(output_dir_ / "overlays");
    fs::create_directories(output_dir_ / "rejected");
  }

  std::vector<Asset> load_assets() const
  {
    return load_asset_manifest(manifest_path_);
  }

  void validate_assets() const
  {
    std::set<std::string> ids;
    std::array<std::map<std::string, int>, 5> counts;
    for (const auto & asset : assets_) {
      if (!ids.insert(asset.id).second) {
        throw std::runtime_error("duplicate asset_id: " + asset.id);
      }
      if (!fs::is_regular_file(asset.sdf_path)) {
        throw std::runtime_error("missing local asset: " + asset.sdf_path.string());
      }
      if (!std::isfinite(asset.z_offset)) {
        throw std::runtime_error("invalid asset z_offset: " + asset.id);
      }
      if (asset.class_id == -1) {
        if (asset.target_class != "distractor" || asset.split != "all") {
          throw std::runtime_error("distractor assets require class_id=-1 and split=all");
        }
        continue;
      }
      if (asset.class_id < 1 || asset.class_id > 5 ||
        asset.target_class != kClassNames.at(static_cast<std::size_t>(asset.class_id - 1)))
      {
        throw std::runtime_error("asset class contract mismatch: " + asset.id);
      }
      if (asset.split != "train" && asset.split != "val" && asset.split != "test") {
        throw std::runtime_error("invalid asset split: " + asset.id);
      }
      ++counts.at(static_cast<std::size_t>(asset.class_id - 1))[asset.split];
    }
    for (std::size_t index = 0; index < counts.size(); ++index) {
      const auto & count = counts[index];
      if (count.count("train") == 0 || count.count("val") == 0 || count.count("test") == 0) {
        throw std::runtime_error(
                std::string(kClassNames[index]) + " must have train, val, and test assets");
      }
    }
  }

  std::string service(const std::string & name) const
  {
    return "/world/" + world_name_ + "/" + name;
  }

  void wait_for_gazebo()
  {
    for (int attempt = 0; attempt < 60 && rclcpp::ok(); ++attempt) {
      std::vector<std::string> services;
      gz_node_.ServiceList(services);
      if (std::find(services.begin(), services.end(), service("create")) != services.end() &&
        std::find(services.begin(), services.end(), service("set_pose")) != services.end() &&
        std::find(services.begin(), services.end(), service("remove")) != services.end())
      {
        return;
      }
      std::this_thread::sleep_for(500ms);
    }
    throw std::runtime_error("Gazebo user command services are unavailable");
  }

  template<typename Request>
  void request_boolean(const std::string & service_name, const Request & request_message) const
  {
    gz::msgs::Boolean response;
    bool service_result = false;
    const bool executed = gz_node_.Request(
      service_name, request_message, 10000, response, service_result);
    if (!executed || !service_result || !response.data()) {
      throw std::runtime_error("Gazebo service failed: " + service_name);
    }
  }

  void remove_spawned_assets()
  {
    const auto entity_names = spawned_entity_names_;
    for (const auto & entity_name : spawned_entity_names_) {
      gz::msgs::Entity request;
      request.set_name(entity_name);
      request.set_type(gz::msgs::Entity::MODEL);
      request_boolean(service("remove"), request);
    }
    if (!entity_names.empty()) {
      std::unique_lock<std::mutex> lock(pose_mutex_);
      const bool removed = pose_condition_.wait_for(
        lock, std::chrono::duration<double>(capture_timeout_), [&]() {
          return std::none_of(
            entity_names.begin(), entity_names.end(),
            [&](const auto & name) {return known_entity_positions_.count(name) != 0;});
        });
      if (!removed) {
        throw std::runtime_error("timed out waiting for previous Gazebo assets to be removed");
      }
    }
    spawned_entity_names_.clear();
  }

  void spawn_asset(
    const Asset & asset, const std::uint64_t sample_seed, const std::size_t ordinal,
    const double x, const double y, const double z, const double yaw)
  {
    const std::string entity_name =
      "dataset_asset_" + std::to_string(sample_seed) + "_" + std::to_string(ordinal);
    gz::msgs::EntityFactory request;
    request.set_name(entity_name);
    request.set_allow_renaming(false);
    request.set_sdf_filename(asset.sdf_path.string());
    request.mutable_pose()->mutable_position()->set_x(x);
    request.mutable_pose()->mutable_position()->set_y(y);
    request.mutable_pose()->mutable_position()->set_z(z + asset.z_offset);
    request.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2.0));
    request_boolean(service("create"), request);
    spawned_entity_names_.push_back(entity_name);
  }

  void set_pose(
    const std::string & entity_name, const double x, const double y,
    const double z, const double yaw) const
  {
    gz::msgs::Pose request;
    request.set_name(entity_name);
    request.mutable_position()->set_x(x);
    request.mutable_position()->set_y(y);
    request.mutable_position()->set_z(z);
    request.mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_orientation()->set_w(std::cos(yaw / 2.0));
    request_boolean(service("set_pose"), request);
  }

  void wait_for_scene_poses(const double robot_x, const double robot_y)
  {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    const bool ready = pose_condition_.wait_for(
      lock, std::chrono::duration<double>(capture_timeout_), [&]() {
        const auto robot = known_entity_positions_.find(robot_name_);
        if (robot == known_entity_positions_.end() ||
        std::abs(robot->second[0] - robot_x) > 0.02 ||
        std::abs(robot->second[1] - robot_y) > 0.02)
        {
          return false;
        }
        return std::all_of(
          spawned_entity_names_.begin(), spawned_entity_names_.end(),
          [&](const auto & name) {return known_entity_positions_.count(name) != 0;});
      });
    if (!ready) {
      throw std::runtime_error("timed out waiting for Gazebo scene poses");
    }
  }

  std::vector<Task> build_tasks() const
  {
    std::vector<std::pair<std::string, int>> split_counts;
    if (negative_only_) {
      split_counts = {{"train", negative_train_count_}, {"val", negative_validation_count_}};
    } else {
      const int train_count = target_count_ * 7 / 10;
      const int val_count = target_count_ / 10;
      split_counts = {{"train", train_count}, {"val", val_count},
        {"test", target_count_ - train_count - val_count}};
    }
    std::vector<Task> tasks;
    for (const auto & [split, count] : split_counts) {
      const int negative_count = negative_only_ ? count : count / 10;
      const int positive_count = count - negative_count;
      for (int index = 0; index < positive_count; ++index) {
        tasks.push_back(
          Task{split, index % 5 + 1, false, false, static_cast<std::size_t>(index / 5)});
      }
      for (int index = 0; index < negative_count; ++index) {
        tasks.push_back(Task{split, 0, true, false, 0});
      }
    }
    std::mt19937 randomizer(static_cast<std::uint32_t>(master_seed_));
    std::vector<std::size_t> background_indices(tasks.size());
    std::iota(background_indices.begin(), background_indices.end(), 0);
    std::shuffle(background_indices.begin(), background_indices.end(), randomizer);
    const std::size_t randomized_count = tasks.size() * 3 / 10;
    for (std::size_t index = 0; index < randomized_count; ++index) {
      tasks[background_indices[index]].has_randomized_background = true;
    }
    std::shuffle(tasks.begin(), tasks.end(), randomizer);
    return tasks;
  }

  json load_state() const
  {
    const fs::path path = output_dir_ / "state.json";
    if (!fs::exists(path)) {
      return {{"accepted", 0}, {"attempts", 0}};
    }
    std::ifstream stream(path);
    json state;
    stream >> state;
    if (state.at("target_count").get<int>() != target_count_ ||
      state.at("seed").get<std::int64_t>() != master_seed_ ||
      state.value("negative_only", false) != negative_only_ ||
      (negative_only_ && (
      state.value("negative_train_count", 0) != negative_train_count_ ||
      state.value("negative_validation_count", 0) != negative_validation_count_)))
    {
      throw std::runtime_error("resume state does not match target_count and seed");
    }
    return state;
  }

  void write_state(const int accepted, const int attempts) const
  {
    const json state = {
      {"target_count", target_count_}, {"seed", master_seed_},
      {"negative_only", negative_only_},
      {"negative_train_count", negative_train_count_},
      {"negative_validation_count", negative_validation_count_},
      {"accepted", accepted}, {"attempts", attempts}};
    const fs::path temporary = output_dir_ / "state.json.tmp";
    std::ofstream(temporary) << state.dump(2) << '\n';
    fs::rename(temporary, output_dir_ / "state.json");
  }

  std::set<std::size_t> build_overlay_indices(const std::vector<Task> & tasks) const
  {
    std::set<std::size_t> indices;
    std::set<std::string> covered_assets;
    std::size_t negative_count = 0;
    for (std::size_t index = 0; index < tasks.size(); ++index) {
      const auto & task = tasks[index];
      if (task.is_negative) {
        if (negative_count++ < 3) {
          indices.insert(index);
        }
      } else if (covered_assets.insert(primary_asset(task).id).second) {
        indices.insert(index);
      }
    }
    return indices;
  }

  const Asset & primary_asset(const Task & task) const
  {
    std::vector<const Asset *> candidates;
    for (const auto & asset : assets_) {
      if (asset.class_id == task.primary_class_id && asset.split == task.split) {
        candidates.push_back(&asset);
      }
    }
    return *candidates.at(round_robin_index(task.primary_asset_ordinal, candidates.size()));
  }

  const Asset * random_asset(
    std::mt19937_64 & randomizer, const std::string & split, const int class_id) const
  {
    std::vector<const Asset *> candidates;
    for (const auto & asset : assets_) {
      if (asset.class_id == class_id && (asset.split == split || asset.split == "all")) {
        candidates.push_back(&asset);
      }
    }
    if (candidates.empty()) {
      throw std::runtime_error("no asset for requested class and split");
    }
    return candidates.at(randomizer() % candidates.size());
  }

  Scene configure_scene(const Task & task, const std::uint64_t sample_seed)
  {
    std::mt19937_64 randomizer(sample_seed);
    remove_spawned_assets();

    const bool randomized_background = task.has_randomized_background;
    const int zone = randomized_background ? 3 + static_cast<int>(randomizer() % 3) :
      static_cast<int>(randomizer() % 3);
    const double zone_x = zone * 10.0;
    constexpr std::array<double, 6> table_x_offsets = {1.2, 1.2, 1.2, 1.2, 1.0, 1.4};
    constexpr std::array<double, 6> table_y_positions = {1.7, 1.7, 1.7, 1.7, 1.25, 1.9};
    const double table_x = zone_x + table_x_offsets.at(static_cast<std::size_t>(zone));
    const double table_y = table_y_positions.at(static_cast<std::size_t>(zone));
    std::uniform_real_distribution<double> x_offset(-0.5, 0.7);
    std::uniform_real_distribution<double> y_offset(-0.7, 0.7);
    std::uniform_real_distribution<double> angle(-3.141592653589793, 3.141592653589793);
    double target_x = zone_x + 1.0 + x_offset(randomizer);
    double target_y = y_offset(randomizer);
    const Asset * primary = nullptr;
    if (!task.is_negative) {
      primary = &primary_asset(task);
      if (primary->surface == "raised") {
        target_x = table_x;
        target_y = table_y;
      }
    }
    const bool is_raised_target = primary && primary->surface == "raised";
    const double distance_min = is_raised_target ? raised_distance_min_ : floor_distance_min_;
    const double distance_max = is_raised_target ? raised_distance_max_ : floor_distance_max_;
    std::uniform_real_distribution<double> distance(distance_min, distance_max);
    const double view_distance = distance(randomizer);
    const double robot_x = target_x - view_distance;
    const double robot_y = target_y;
    const double robot_yaw = 0.0;
    set_pose(robot_name_, robot_x, robot_y, 0.01, robot_yaw);

    Scene scene{sample_seed, randomized_background ? "randomized" : "house_like", zone,
      robot_x, robot_y, robot_yaw, {}, task.primary_class_id};
    if (!task.is_negative) {
      const double target_z = primary->surface == "raised" ? 0.24 : 0.02;
      spawn_asset(
        *primary, sample_seed, scene.active_assets.size(), target_x, target_y, target_z,
        angle(randomizer));
      scene.active_assets.push_back(primary);

      const int additional_count = static_cast<int>(randomizer() % 4);
      for (int additional = 0; additional < additional_count; ++additional) {
        const bool has_distractor = std::any_of(assets_.begin(), assets_.end(),
            [](const auto & item) {
              return item.class_id == -1;
          });
        const bool use_distractor = has_distractor && randomizer() % 3 == 0;
        const int class_id = use_distractor ? -1 :
          (task.primary_class_id + static_cast<int>(randomizer() % 4)) % 5 + 1;
        const Asset * asset = random_asset(randomizer, task.split, class_id);
        if (std::find(scene.active_assets.begin(), scene.active_assets.end(), asset) !=
          scene.active_assets.end())
        {
          continue;
        }
        const double offset = 0.25 + additional * 0.18;
        const double asset_z = asset->surface == "raised" ? 0.24 : 0.02;
        const double asset_x = target_x + offset;
        const double asset_y = target_y + (additional % 2 == 0 ? 0.18 : -0.18);
        spawn_asset(
          *asset, sample_seed, scene.active_assets.size(), asset_x, asset_y, asset_z,
          angle(randomizer));
        scene.active_assets.push_back(asset);
      }
    } else {
      std::vector<const Asset *> distractors;
      for (const auto & asset : assets_) {
        if (asset.class_id == -1) {
          distractors.push_back(&asset);
        }
      }
      const int count = distractors.empty() ? 0 : static_cast<int>(randomizer() % 4);
      for (int index = 0; index < count; ++index) {
        const Asset * asset = distractors.at(randomizer() % distractors.size());
        if (std::find(scene.active_assets.begin(), scene.active_assets.end(), asset) !=
          scene.active_assets.end())
        {
          continue;
        }
        spawn_asset(
          *asset, sample_seed, scene.active_assets.size(), target_x + index * 0.25, target_y,
          0.02, angle(randomizer));
        scene.active_assets.push_back(asset);
      }
    }

    wait_for_scene_poses(robot_x, robot_y);

    std::lock_guard<std::mutex> lock(frame_mutex_);
    capture_after_rgbd_sequence_ = rgbd_sequence_ + static_cast<std::uint64_t>(settle_frames_);
    capture_after_label_sequence_ = label_sequence_ + static_cast<std::uint64_t>(settle_frames_);
    return scene;
  }

  bool contains_class_label(const int class_id) const
  {
    if (label_step_ < label_width_ * 3 ||
      latest_labels_.size() < static_cast<std::size_t>(label_step_) * label_height_)
    {
      return false;
    }
    for (std::uint32_t y = 0; y < label_height_; ++y) {
      for (std::uint32_t x = 0; x < label_width_; ++x) {
        const std::size_t offset = static_cast<std::size_t>(y) * label_step_ + x * 3;
        if (latest_labels_[offset + 2] == class_id) {
          return true;
        }
      }
    }
    return false;
  }

  Frame wait_for_frame(const Task & task)
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    const bool ready = frame_condition_.wait_for(
      lock, std::chrono::duration<double>(capture_timeout_), [this, &task]() {
        if (!latest_color_ || !latest_depth_ || latest_labels_.empty() ||
        rgbd_sequence_ < capture_after_rgbd_sequence_ ||
        label_sequence_ < capture_after_label_sequence_)
        {
          return false;
        }
        const auto color_time = ros_stamp(latest_color_->header.stamp);
        const bool synchronized = color_time > 0 && label_stamp_ > 0 &&
        std::llabs(color_time - label_stamp_) <= 50000000LL;
        return synchronized &&
               (task.is_negative || contains_class_label(task.primary_class_id));
      });
    if (!ready) {
      const auto color_time = latest_color_ ? ros_stamp(latest_color_->header.stamp) : 0;
      std::ostringstream reason;
      reason << "timed out waiting for synchronized RGB-D and expected labels"
             << "; rgbd_sequence=" << rgbd_sequence_ << "/" << capture_after_rgbd_sequence_
             << ",label_sequence=" << label_sequence_ << "/" << capture_after_label_sequence_
             << ",stamp_delta_ns=" << std::llabs(color_time - label_stamp_)
             << ",primary_label_present="
             << (task.is_negative || contains_class_label(task.primary_class_id));
      throw std::runtime_error(reason.str());
    }
    return Frame{latest_color_, latest_depth_, latest_labels_, label_width_, label_height_,
      label_step_, ros_stamp(latest_color_->header.stamp), label_stamp_};
  }

  void save_sample(
    const int index, const Task & task, const Scene & scene, const Frame & frame,
    const bool save_overlay) const
  {
    cv::Mat color = cv_bridge::toCvCopy(
      frame.color, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat depth = cv_bridge::toCvCopy(
      frame.depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    if (color.cols != 640 || color.rows != 480 || depth.size() != color.size() ||
      frame.label_width != 640 || frame.label_height != 480)
    {
      throw std::runtime_error("camera outputs must all be 640x480");
    }
    const auto mean = cv::mean(color);
    if (mean[0] + mean[1] + mean[2] < 6.0) {
      throw std::runtime_error("RGB image is blank");
    }
    const auto boxes = decode_instance_boxes(
      frame.labels, frame.label_width, frame.label_height, frame.label_step,
      static_cast<std::size_t>(minimum_visible_pixels_), minimum_box_dimension_);
    const auto oversized = std::find_if(boxes.begin(), boxes.end(), [&](const auto & box) {
          return is_box_oversized(
          box, frame.label_width, frame.label_height, maximum_box_area_ratio_);
      });
    if (oversized != boxes.end()) {
      const double box_area =
        static_cast<double>(oversized->max_x - oversized->min_x + 1) *
        (oversized->max_y - oversized->min_y + 1);
      std::ostringstream reason;
      reason << "oversized_box: class=" << static_cast<int>(oversized->class_id)
             << ",area_ratio=" << box_area / (frame.label_width * frame.label_height)
             << ",maximum=" << maximum_box_area_ratio_;
      throw std::runtime_error(reason.str());
    }
    if (task.is_negative && !boxes.empty()) {
      throw std::runtime_error("negative scene contains labeled target pixels");
    }
    if (!task.is_negative && std::none_of(boxes.begin(), boxes.end(), [&](const auto & box) {
        return box.class_id == task.primary_class_id;
      }))
    {
      const auto raw_boxes = decode_instance_boxes(
        frame.labels, frame.label_width, frame.label_height, frame.label_step, 1, 1);
      std::ostringstream reason;
      reason << "primary target is not sufficiently visible; decoded labels:";
      for (const auto & box : raw_boxes) {
        reason << " class=" << static_cast<int>(box.class_id)
               << ",pixels=" << box.visible_pixels;
      }
      if (raw_boxes.empty()) {
        reason << " none";
      }
      const std::string diagnostic = "seed_" + std::to_string(scene.seed);
      cv::imwrite((output_dir_ / "rejected" / (diagnostic + "_rgb.jpg")).string(), color);
      cv::Mat rejected_labels_rgb(
        static_cast<int>(frame.label_height), static_cast<int>(frame.label_width), CV_8UC3,
        const_cast<std::uint8_t *>(frame.labels.data()), frame.label_step);
      cv::Mat rejected_labels_bgr;
      cv::cvtColor(rejected_labels_rgb, rejected_labels_bgr, cv::COLOR_RGB2BGR);
      cv::imwrite(
        (output_dir_ / "rejected" / (diagnostic + "_labels.png")).string(),
        rejected_labels_bgr);
      throw std::runtime_error(reason.str());
    }

    std::ostringstream name_stream;
    name_stream << "gz_" << std::setw(6) << std::setfill('0') << index;
    const std::string name = name_stream.str();
    const fs::path image_path = output_dir_ / "images" / task.split / (name + ".jpg");
    const fs::path label_path = output_dir_ / "labels" / task.split / (name + ".txt");
    const fs::path depth_path = output_dir_ / "depth" / task.split / (name + ".png");
    const fs::path instance_path = output_dir_ / "instance" / task.split / (name + ".png");
    const fs::path metadata_path = output_dir_ / "metadata" / task.split / (name + ".json");

    if (!cv::imwrite(image_path.string(), color, {cv::IMWRITE_JPEG_QUALITY, 95})) {
      throw std::runtime_error("failed to save RGB image");
    }
    cv::Mat depth_mm(depth.size(), CV_16UC1, cv::Scalar(0));
    for (int y = 0; y < depth.rows; ++y) {
      for (int x = 0; x < depth.cols; ++x) {
        const float meters = depth.at<float>(y, x);
        if (std::isfinite(meters) && meters > 0.0F && meters < 65.535F) {
          depth_mm.at<std::uint16_t>(y, x) = static_cast<std::uint16_t>(std::round(meters * 1000));
        }
      }
    }
    cv::imwrite(depth_path.string(), depth_mm);
    cv::Mat labels_rgb(
      static_cast<int>(frame.label_height), static_cast<int>(frame.label_width), CV_8UC3,
      const_cast<std::uint8_t *>(frame.labels.data()), frame.label_step);
    cv::Mat labels_bgr;
    cv::cvtColor(labels_rgb, labels_bgr, cv::COLOR_RGB2BGR);
    cv::imwrite(instance_path.string(), labels_bgr);

    std::ofstream labels(label_path);
    json box_metadata = json::array();
    for (const auto & box : boxes) {
      labels << to_yolo_line(box, frame.label_width, frame.label_height) << '\n';
      box_metadata.push_back({
          {"class_id", box.class_id - 1}, {"instance_id", box.instance_id},
          {"visible_pixels", box.visible_pixels},
          {"xyxy", {box.min_x, box.min_y, box.max_x, box.max_y}},
          {"touches_edge", box.min_x == 0 || box.min_y == 0 ||
            box.max_x == static_cast<int>(frame.label_width) - 1 ||
            box.max_y == static_cast<int>(frame.label_height) - 1}});
    }
    json active_assets = json::array();
    for (const auto * asset : scene.active_assets) {
      active_assets.push_back({
          {"asset_id", asset->id}, {"class", asset->target_class}, {"color", asset->color},
          {"z_offset", asset->z_offset}, {"source_url", asset->source_url}});
    }
    const json metadata = {
      {"name", name}, {"split", task.split}, {"sample_seed", scene.seed},
      {"negative", task.is_negative},
      {"primary_class", task.is_negative ? "none" : kClassNames.at(task.primary_class_id - 1)},
      {"primary_asset_id", task.is_negative ? "none" : scene.active_assets.front()->id},
      {"background_mode", scene.background_mode}, {"zone", scene.zone},
      {"robot_pose", {scene.robot_x, scene.robot_y, scene.robot_yaw}},
      {"rgb_stamp_ns", frame.color_stamp}, {"label_stamp_ns", frame.label_stamp},
      {"assets", active_assets}, {"boxes", box_metadata}};
    std::ofstream(metadata_path) << metadata.dump(2) << '\n';

    if (save_overlay) {
      cv::Mat overlay = color.clone();
      for (const auto & box : boxes) {
        cv::rectangle(
          overlay, cv::Point(box.min_x, box.min_y), cv::Point(box.max_x, box.max_y),
          cv::Scalar(0, 255, 0), 2);
        cv::putText(
          overlay, kClassNames.at(box.class_id - 1), cv::Point(box.min_x, std::max(14, box.min_y)),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
      }
      cv::imwrite((output_dir_ / "overlays" / (name + ".jpg")).string(), overlay);
    }
  }

  void write_rejection(
    const int attempt, const std::uint64_t sample_seed, const Task & task,
    const std::optional<Scene> & scene, const std::string & reason) const
  {
    json assets = json::array();
    if (scene) {
      for (const auto * asset : scene->active_assets) {
        assets.push_back({
            {"asset_id", asset->id}, {"class_id", asset->class_id},
            {"surface", asset->surface}, {"z_offset", asset->z_offset}});
      }
    }
    const json rejection = {
      {"attempt", attempt}, {"sample_seed", sample_seed}, {"split", task.split},
      {"primary_class_id", task.primary_class_id}, {"negative", task.is_negative},
      {"assets", assets},
      {"reason", reason}};
    std::ofstream(output_dir_ / "rejected" / ("attempt_" + std::to_string(attempt) + ".json"))
      << rejection.dump(2) << '\n';
  }

  void write_report() const
  {
    std::map<std::string, int> split_counts;
    std::map<std::string, int> background_counts;
    std::map<std::string, int> primary_asset_counts;
    for (const auto & asset : assets_) {
      if (!negative_only_ && asset.class_id > 0) {
        primary_asset_counts[asset.id] = 0;
      }
    }
    std::array<int, 5> class_counts{};
    int negative_count = 0;
    int box_count = 0;
    int edge_count = 0;
    std::size_t minimum_pixels = std::numeric_limits<std::size_t>::max();
    std::size_t maximum_pixels = 0;
    for (const auto * split : {"train", "val", "test"}) {
      for (const auto & entry : fs::directory_iterator(output_dir_ / "metadata" / split)) {
        std::ifstream stream(entry.path());
        json metadata;
        stream >> metadata;
        ++split_counts[split];
        ++background_counts[metadata.at("background_mode").get<std::string>()];
        const bool is_negative = metadata.at("negative").get<bool>();
        negative_count += is_negative ? 1 : 0;
        if (!is_negative) {
          ++primary_asset_counts.at(metadata.at("primary_asset_id").get<std::string>());
        }
        for (const auto & box : metadata.at("boxes")) {
          ++box_count;
          ++class_counts.at(box.at("class_id").get<std::size_t>());
          edge_count += box.at("touches_edge").get<bool>() ? 1 : 0;
          const auto pixels = box.at("visible_pixels").get<std::size_t>();
          minimum_pixels = std::min(minimum_pixels, pixels);
          maximum_pixels = std::max(maximum_pixels, pixels);
        }
      }
    }
    int rejected_count = 0;
    for (const auto & entry : fs::directory_iterator(output_dir_ / "rejected")) {
      rejected_count += entry.path().extension() == ".json" ? 1 : 0;
    }
    std::vector<std::string> missing_primary_assets;
    if (!negative_only_) {
      for (const auto & [asset_id, count] : primary_asset_counts) {
        if (count == 0) {
          missing_primary_assets.push_back(asset_id);
        }
      }
    }
    const double rejection_ratio = static_cast<double>(rejected_count) / target_count_;
    json classes;
    for (std::size_t index = 0; index < class_counts.size(); ++index) {
      classes[kClassNames[index]] = class_counts[index];
    }
    json asset_licenses = json::array();
    for (const auto & asset : assets_) {
      asset_licenses.push_back({
          {"asset_id", asset.id}, {"source_url", asset.source_url},
          {"license", asset.license}, {"license_url", asset.license_url},
          {"sha256", asset.sha256}, {"z_offset", asset.z_offset}});
    }
    const json report = {
      {"target_count", target_count_}, {"seed", master_seed_},
      {"negative_only", negative_only_},
      {"splits", split_counts}, {"negative_images", negative_count},
      {"background_modes", background_counts},
      {"class_annotations", classes}, {"box_count", box_count},
      {"edge_boxes", edge_count}, {"minimum_visible_pixels", box_count ? minimum_pixels : 0},
      {"maximum_visible_pixels", maximum_pixels}, {"rejected_attempts", rejected_count},
      {"rejection_ratio", rejection_ratio},
      {"maximum_rejection_ratio", maximum_rejection_ratio_},
      {"distance_ranges_m", {
          {"raised", {{"min", raised_distance_min_}, {"max", raised_distance_max_}}},
          {"floor", {{"min", floor_distance_min_}, {"max", floor_distance_max_}}}}},
      {"primary_asset_counts", primary_asset_counts},
      {"missing_primary_assets", missing_primary_assets},
      {"maximum_allowed_box_area_ratio", maximum_box_area_ratio_},
      {"assets", asset_licenses}};
    std::ofstream(output_dir_ / "report.json") << report.dump(2) << '\n';

    std::ofstream markdown(output_dir_ / "report.md");
    markdown  << "# Gazebo GSO 数据质量报告\n\n"
              << "- 有效图片：" << target_count_ << "\n"
              << "- 划分：train=" << split_counts["train"] << "，val=" << split_counts["val"]
              << "，test=" << split_counts["test"] << "\n"
              << "- 背景：house_like=" << background_counts["house_like"]
              << "，randomized=" << background_counts["randomized"] << "\n"
              << "- 纯负样本：" << negative_count << "\n"
              << "- 有效检测框：" << box_count << "\n"
              << "- 接触图像边缘的框：" << edge_count << "\n"
              << "- 最大允许框面积比例：" << maximum_box_area_ratio_ << "\n"
              << "- raised 相机距离：" << raised_distance_min_ << "–" << raised_distance_max_ << " m\n"
              << "- floor 相机距离：" << floor_distance_min_ << "–" << floor_distance_max_ << " m\n"
              << "- 被拒绝尝试：" << rejected_count << "（比例=" << rejection_ratio << "）\n"
              << "- 主目标模型遗漏：" << missing_primary_assets.size() << "\n\n"
              << "## 类别标注数\n\n";
    for (std::size_t index = 0; index < class_counts.size(); ++index) {
      markdown << "- " << kClassNames[index] << "：" << class_counts[index] << "\n";
    }
    markdown << "\n## 主目标模型次数\n\n";
    for (const auto & [asset_id, count] : primary_asset_counts) {
      markdown << "- " << asset_id << "：" << count << "\n";
    }
    markdown << "\n## 人工门禁\n\n";
    if (negative_only_) {
      markdown << "检查困难负样本审查清单，确认图片中没有 cup、backpack、ball 或 box。\n";
    } else {
      markdown << "检查 `overlays/` 中每个模型至少一张主目标图，并检查 3 张负样本。\n";
    }

    if (require_full_primary_coverage_ && !missing_primary_assets.empty()) {
      throw std::runtime_error("primary asset coverage gate failed");
    }
    if (maximum_rejection_ratio_ >= 0.0 && rejection_ratio > maximum_rejection_ratio_) {
      throw std::runtime_error("rejection ratio gate failed");
    }
  }

  fs::path output_dir_;
  fs::path manifest_path_;
  fs::path approval_file_;
  int target_count_;
  std::int64_t master_seed_;
  int settle_frames_;
  double capture_timeout_;
  double raised_distance_min_;
  double raised_distance_max_;
  double floor_distance_min_;
  double floor_distance_max_;
  int minimum_visible_pixels_;
  int minimum_box_dimension_;
  double maximum_box_area_ratio_;
  int maximum_consecutive_failures_;
  bool require_full_primary_coverage_;
  double maximum_rejection_ratio_;
  bool negative_only_;
  int negative_train_count_;
  int negative_validation_count_;
  std::string robot_name_;
  std::string world_name_;
  std::string label_topic_;
  std::string pose_topic_;
  std::vector<Asset> assets_;
  std::vector<std::string> spawned_entity_names_;

  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  mutable gz::transport::Node gz_node_;
  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  Image::ConstSharedPtr latest_color_;
  Image::ConstSharedPtr latest_depth_;
  std::vector<std::uint8_t> latest_labels_;
  std::uint32_t label_width_{0};
  std::uint32_t label_height_{0};
  std::uint32_t label_step_{0};
  std::int64_t label_stamp_{0};
  std::uint64_t rgbd_sequence_{0};
  std::uint64_t label_sequence_{0};
  std::uint64_t capture_after_rgbd_sequence_{0};
  std::uint64_t capture_after_label_sequence_{0};
  std::mutex pose_mutex_;
  std::condition_variable pose_condition_;
  std::unordered_map<std::string, std::array<double, 2>> known_entity_positions_;
};

}  // namespace turtlebot3_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<turtlebot3_gazebo::GsoDatasetGenerator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});
    std::exception_ptr run_error;
    try {
      node->run();
    } catch (...) {
      run_error = std::current_exception();
    }
    node->stop_transport();
    executor.cancel();
    spin_thread.join();
    if (run_error) {
      std::rethrow_exception(run_error);
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gso_dataset_generator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
