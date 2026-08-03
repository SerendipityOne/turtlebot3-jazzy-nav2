// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <cv_bridge/cv_bridge.hpp>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "turtlebot3_embodied_navigation/skill_policy_client.hpp"
#include "turtlebot3_embodied_navigation/vla_policy_state.hpp"

namespace turtlebot3_gazebo
{

namespace fs = std::filesystem;
using Image = sensor_msgs::msg::Image;
using json = nlohmann::json;
using turtlebot3_embodied_navigation::VLAAction;
using turtlebot3_embodied_navigation::ContinuousVLAAction;
using turtlebot3_embodied_navigation::ContinuousVLAState;
using turtlebot3_embodied_navigation::VLAState;
using turtlebot3_embodied_navigation::VLAViewpoint;
using turtlebot3_embodied_navigation::build_continuous_vla_policy_state;
using turtlebot3_embodied_navigation::build_vla_policy_state;
using turtlebot3_embodied_navigation::kVlaCandidateSlots;

void write_text_file(const fs::path & path, const std::string & content)
{
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << content;
  if (!output) {
    throw std::runtime_error("failed while writing " + path.string());
  }
}

namespace
{

constexpr std::array<const char *, 4> kColors{"red", "green", "blue", "yellow"};
constexpr double kScanOffset = 0.78539816339744830962;
constexpr double kMaxLocalGoalDistanceM = 0.75;
constexpr double kMaxLocalGoalYawRad = 0.78539816339744830962;

std::string split_for_episode(const int index, const int total)
{
  const int train_end = total * 70 / 100;
  const int validation_end = total * 85 / 100;
  return index < train_end ? "train" : (index < validation_end ? "validation" : "test");
}

int approach_action(const std::string & color)
{
  const auto iterator = std::find(kColors.begin(), kColors.end(), color);
  if (iterator == kColors.end()) {
    throw std::invalid_argument("unknown ball color");
  }
  return 5 + static_cast<int>(iterator - kColors.begin());
}

std::string instruction_for(const std::string & color, const int variant)
{
  static constexpr std::array<const char *, 8> kTemplates{
    "找{color}球", "请找到{color}色的球", "去找一个{color}球", "定位{color}球",
    "帮我寻找{color}色球", "请搜索{color}颜色的球", "我要找{color}球", "找到那个{color}球"};
  std::string instruction = kTemplates.at(static_cast<std::size_t>(variant) % kTemplates.size());
  const auto marker = instruction.find("{color}");
  instruction.replace(marker, 7, color == "red" ? "红" : color == "green" ? "绿" :
    color == "blue" ? "蓝" : "黄");
  return instruction;
}

std::string stage_instruction_for(const std::string & color, const bool target_visible)
{
  return std::string(target_visible ? "approach for " : "search for ") + color + " ball";
}

double normalize_angle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 6.28318530717958647692;
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

std::string ball_sdf(const std::string & color, const int semantic_label)
{
  const std::string diffuse = color == "red" ? "0.9 0.05 0.05 1" :
    color == "green" ? "0.05 0.8 0.05 1" : color == "blue" ? "0.05 0.2 0.9 1" :
    "0.9 0.8 0.05 1";
  // The Gazebo Label system, not visual color, defines Oracle ground truth.
  return "<sdf version='1.9'><model name='oracle_ball'><static>true</static>"
         "<plugin filename='gz-sim-label-system' name='gz::sim::systems::Label'><label>" +
         std::to_string(semantic_label) + "</label></plugin>"
         "<link name='link'><collision name='collision'><geometry><sphere><radius>0.10</radius>"
         "</sphere></geometry></collision><visual name='visual'><geometry><sphere><radius>0.10</radius>"
         "</sphere></geometry><material><diffuse>" + diffuse + "</diffuse><ambient>" + diffuse +
         "</ambient></material></visual></link></model></sdf>";
}

}  // namespace

class VLAOracleDatasetGenerator : public rclcpp::Node
{
public:
  VLAOracleDatasetGenerator()
  : Node("vla_oracle_dataset_generator")
  {
    output_dir_ = fs::absolute(declare_parameter<std::string>("output_dir"));
    approval_file_ = fs::absolute(declare_parameter<std::string>("approval_file", "approval.txt"));
    episode_count_ = declare_parameter("episode_count", 200);
    master_seed_ = declare_parameter<std::int64_t>("seed", 20260727);
    target_absent_ratio_ = declare_parameter("target_absent_ratio", 0.30);
    policy_interface_ = declare_parameter<std::string>("policy_interface", "discrete_skill");
    resume_ = declare_parameter("resume", false);
    settle_frames_ = declare_parameter("settle_frames", 8);
    capture_timeout_ = declare_parameter("capture_timeout", 20.0);
    world_name_ = declare_parameter<std::string>("world_name", "default");
    robot_name_ = declare_parameter<std::string>("robot_name", "waffle_pi_cam");
    label_topic_ = declare_parameter<std::string>(
      "label_topic", "/dataset/segmentation/labels_map");
    viewpoints_ = declare_parameter<std::vector<double>>(
      "search_viewpoints", std::vector<double>{
        1.083, 1.090, 0.72, 0.855, 4.872, 3.14, -1.866, 4.661, 3.02,
        -4.151, 4.392, -1.60, -4.316, 0.217, -1.60, -4.228, -2.601, -2.90,
        -4.922, -1.308, 1.33, -3.236, 3.217, 0.00, -0.691, 2.877, -1.27,
        0.004, 0.989, 0.00});
    color_sub_ = create_subscription<Image>(
      declare_parameter<std::string>("color_topic", "/camera/color/image_raw"),
      rclcpp::SensorDataQoS(), std::bind(&VLAOracleDatasetGenerator::on_image, this, std::placeholders::_1));
    if (output_dir_.empty() || episode_count_ <= 0 ||
      settle_frames_ <= 0 || capture_timeout_ <= 0.0 ||
      target_absent_ratio_ < 0.0 || target_absent_ratio_ > 1.0 || viewpoints_.empty() ||
      viewpoints_.size() % 3 != 0)
    {
      throw std::invalid_argument("VLA oracle generator parameters are invalid");
    }
    if (policy_interface_ != "discrete_skill" && policy_interface_ != "continuous_local_goal") {
      throw std::invalid_argument("policy_interface must be discrete_skill or continuous_local_goal");
    }
    if (!gz_node_.Subscribe(label_topic_, &VLAOracleDatasetGenerator::on_labels, this)) {
      throw std::runtime_error("failed to subscribe to Gazebo labels: " + label_topic_);
    }
  }

  void run()
  {
    require_approval();
    if (fs::exists(output_dir_) && !resume_) {
      throw std::runtime_error("output directory already exists: " + output_dir_.string());
    }
    if (fs::exists(output_dir_) && !fs::is_directory(output_dir_)) {
      throw std::runtime_error("output path is not a directory: " + output_dir_.string());
    }
    fs::create_directories(output_dir_ / "episodes");
    wait_for_gazebo();

    bool found_missing_episode = false;
    for (int episode_index = 0; episode_index < episode_count_ && rclcpp::ok(); ++episode_index) {
      const fs::path episode_dir = output_dir_ / "episodes" /
        ("episode_" + std::to_string(episode_index));
      if (fs::exists(episode_dir)) {
        if (!resume_) {
          throw std::runtime_error("episode directory already exists: " + episode_dir.string());
        }
        if (found_missing_episode) {
          throw std::runtime_error("resume found a completed episode after a missing index: " +
            episode_dir.string());
        }
        if (!is_complete_episode(episode_dir, episode_index)) {
          throw std::runtime_error("resume found an incomplete episode directory: " +
            episode_dir.string());
        }
        RCLCPP_INFO(get_logger(), "Resuming: keeping completed VLA oracle episode %d/%d",
          episode_index + 1, episode_count_);
        continue;
      }
      found_missing_episode = true;
      generate_episode(episode_index);
    }
    if (!rclcpp::ok()) {
      throw std::runtime_error("VLA oracle generation was interrupted");
    }
    write_generation_report();
  }

  void stop_transport()
  {
    gz_node_.Unsubscribe(label_topic_);
  }

private:
  struct CapturedFrame
  {
    Image::ConstSharedPtr image;
    std::vector<std::uint8_t> labels;
    std::uint32_t label_width;
    std::uint32_t label_height;
    std::uint32_t label_step;
  };

  void on_image(const Image::ConstSharedPtr image)
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_image_ = image;
    ++image_sequence_;
    frame_condition_.notify_all();
  }

  void on_labels(const gz::msgs::Image & image)
  {
    if (image.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
      return;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_labels_.assign(image.data().begin(), image.data().end());
    label_width_ = image.width();
    label_height_ = image.height();
    label_step_ = image.step();
    ++label_sequence_;
    frame_condition_.notify_all();
  }

  std::string service(const std::string & name) const
  {
    return "/world/" + world_name_ + "/" + name;
  }

  bool is_complete_episode(const fs::path & episode_dir, const int episode_index) const
  {
    try {
      std::ifstream metadata_file(episode_dir / "episode.json");
      if (!metadata_file) {
        return false;
      }
      const json metadata = json::parse(metadata_file);
      const std::uint64_t expected_seed = static_cast<std::uint64_t>(master_seed_) * 1000003ULL +
        static_cast<std::uint64_t>(episode_index);
      if (metadata.value("episode_id", -1) != episode_index ||
        metadata.value("seed", std::uint64_t{0}) != expected_seed ||
        metadata.value("split", "") != split_for_episode(episode_index, episode_count_) ||
        metadata.value("policy_interface", "discrete_skill") != policy_interface_)
      {
        return false;
      }
      std::ifstream steps(episode_dir / "steps.jsonl");
      if (!steps) {
        return false;
      }
      std::string line;
      bool has_steps = false;
      while (std::getline(steps, line)) {
        if (line.empty()) {
          continue;
        }
        const json step = json::parse(line);
        const std::string image = step.at("image").get<std::string>();
        if (!fs::is_regular_file(episode_dir / image)) {
          return false;
        }
        has_steps = true;
      }
      return has_steps && steps.eof();
    } catch (const std::exception &) {
      return false;
    }
  }

  void require_approval() const
  {
    if (episode_count_ <= 200) {
      return;
    }
    std::ifstream input(approval_file_);
    std::string line;
    std::getline(input, line);
    if (line != "APPROVED") {
      throw std::runtime_error("episode_count above 200 requires approval_file containing APPROVED");
    }
  }

  void wait_for_gazebo() const
  {
    for (int attempt = 0; attempt < 60 && rclcpp::ok(); ++attempt) {
      std::vector<std::string> services;
      gz_node_.ServiceList(services);
      if (std::find(services.begin(), services.end(), service("create")) != services.end() &&
        std::find(services.begin(), services.end(), service("remove")) != services.end() &&
        std::find(services.begin(), services.end(), service("set_pose")) != services.end())
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    throw std::runtime_error("Gazebo create, remove and set_pose services are unavailable");
  }

  template<typename Request>
  void request_boolean(const std::string & service_name, const Request & request)
  {
    gz::msgs::Boolean response;
    bool service_result = false;
    const bool executed = gz_node_.Request(service_name, request, 10000, response, service_result);
    if (!executed || !service_result || !response.data()) {
      throw std::runtime_error("Gazebo service failed: " + service_name);
    }
  }

  void set_robot_pose(const VLAViewpoint & pose, const double yaw)
  {
    gz::msgs::Pose request;
    request.set_name(robot_name_);
    request.mutable_position()->set_x(pose.x);
    request.mutable_position()->set_y(pose.y);
    request.mutable_position()->set_z(0.01);
    request.mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_orientation()->set_w(std::cos(yaw / 2.0));
    request_boolean(service("set_pose"), request);
  }

  void spawn_ball(
    const std::string & name, const std::string & color, const VLAViewpoint & viewpoint,
    const std::size_t position, const double offset)
  {
    const double yaw = viewpoint.yaw + (position % 2 == 0 ? 0.20 : -0.20);
    gz::msgs::EntityFactory request;
    request.set_name(name);
    request.set_allow_renaming(false);
    request.set_sdf(ball_sdf(color, approach_action(color) - 4));
    request.mutable_pose()->mutable_position()->set_x(viewpoint.x + offset * std::cos(yaw));
    request.mutable_pose()->mutable_position()->set_y(viewpoint.y + offset * std::sin(yaw));
    request.mutable_pose()->mutable_position()->set_z(0.10);
    request_boolean(service("create"), request);
    spawned_entities_.push_back(name);
  }

  void remove_balls()
  {
    for (const auto & name : spawned_entities_) {
      gz::msgs::Entity request;
      request.set_name(name);
      request.set_type(gz::msgs::Entity::MODEL);
      request_boolean(service("remove"), request);
    }
    spawned_entities_.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  CapturedFrame capture_frame()
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    const std::size_t target_image_sequence = image_sequence_ + static_cast<std::size_t>(settle_frames_);
    const std::size_t target_label_sequence = label_sequence_ + static_cast<std::size_t>(settle_frames_);
    // A fresh RGB frame with a stale label map would silently corrupt Oracle actions.
    const bool captured = frame_condition_.wait_for(
      lock, std::chrono::duration<double>(capture_timeout_), [&]() {
        return latest_image_ && !latest_labels_.empty() && image_sequence_ >= target_image_sequence &&
               label_sequence_ >= target_label_sequence &&
               label_width_ > 0 && label_height_ > 0 && label_step_ >= label_width_ * 3;
      });
    if (!captured) {
      throw std::runtime_error("timed out waiting for RGB and segmentation frames");
    }
    return CapturedFrame{latest_image_, latest_labels_, label_width_, label_height_, label_step_};
  }

  static std::size_t count_semantic_label(const CapturedFrame & frame, const std::uint8_t label)
  {
    std::size_t count = 0;
    for (std::uint32_t y = 0; y < frame.label_height; ++y) {
      for (std::uint32_t x = 0; x < frame.label_width; ++x) {
        if (frame.labels.at(static_cast<std::size_t>(y) * frame.label_step + x * 3 + 2) == label) {
          ++count;
        }
      }
    }
    return count;
  }

  static int select_candidate_action(const VLAState & state)
  {
    double best_cost = std::numeric_limits<double>::infinity();
    int best_slot = -1;
    for (std::size_t slot = 0; slot < kVlaCandidateSlots; ++slot) {
      const std::size_t offset = slot * 4;
      if (state.values[offset + 3] == 0.0) {
        continue;
      }
      const double bearing = std::atan2(state.values[offset + 1], state.values[offset + 2]);
      const double cost = state.values[offset] + 0.5 * std::abs(bearing);
      if (cost < best_cost) {
        best_cost = cost;
        best_slot = static_cast<int>(slot);
      }
    }
    return best_slot;
  }

  void save_step(
    const fs::path & episode_dir, int & frame_index, const CapturedFrame & capture,
    const std::string & instruction, const VLAState & state, const int action_id,
    const std::size_t target_label_pixels, const bool coverage_complete, std::ofstream & steps)
  {
    const std::string file_name = "frames/" + std::to_string(frame_index++) + ".jpg";
    const auto image = cv_bridge::toCvCopy(capture.image, "bgr8");
    if (!cv::imwrite((episode_dir / file_name).string(), image->image)) {
      throw std::runtime_error("failed to write oracle RGB frame");
    }
    steps << json{
      {"image", file_name},
      {"instruction", instruction},
      {"robot_state", state.values},
      {"action_id", action_id},
      {"oracle", {{"target_visible", target_label_pixels > 0},
                    {"target_label_pixels", target_label_pixels},
                    {"coverage_complete", coverage_complete}}}
    }.dump() << '\n';
  }

  void generate_episode(const int episode_index)
  {
    if (policy_interface_ == "continuous_local_goal") {
      generate_continuous_episode(episode_index);
      return;
    }
    remove_balls();
    const std::uint64_t seed = static_cast<std::uint64_t>(master_seed_) * 1000003ULL + episode_index;
    std::mt19937_64 randomizer(seed);
    const std::string color = kColors.at(randomizer() % kColors.size());
    const bool target_present = std::uniform_real_distribution<double>(0.0, 1.0)(randomizer) >=
      target_absent_ratio_;
    const std::string split = split_for_episode(episode_index, episode_count_);
    std::vector<VLAViewpoint> viewpoints;
    viewpoints.reserve(viewpoints_.size() / 3);
    for (std::size_t index = 0; index < viewpoints_.size() / 3; ++index) {
      viewpoints.push_back(VLAViewpoint{
        index, viewpoints_[index * 3], viewpoints_[index * 3 + 1], viewpoints_[index * 3 + 2]});
    }
    const std::size_t target_viewpoint = randomizer() % viewpoints.size();
    if (target_present) {
      spawn_ball("oracle_target_" + std::to_string(seed), color, viewpoints[target_viewpoint], 0, 0.90);
    }
    const int distractor_count = 1 + static_cast<int>(randomizer() % 3);
    const auto target_color_iterator = std::find(kColors.begin(), kColors.end(), color);
    const std::size_t target_color_index = static_cast<std::size_t>(
      std::distance(kColors.begin(), target_color_iterator));
    for (int ordinal = 0; ordinal < distractor_count; ++ordinal) {
      const std::string distractor_color = target_present ?
        kColors.at(randomizer() % kColors.size()) :
        kColors.at((target_color_index + static_cast<std::size_t>(ordinal) + 1U) % kColors.size());
      const std::size_t position = (target_viewpoint + 1 + ordinal * 3) % viewpoints.size();
      spawn_ball(
        "oracle_distractor_" + std::to_string(seed) + "_" + std::to_string(ordinal),
        distractor_color, viewpoints[position], static_cast<std::size_t>(ordinal + 1), 0.75);
    }

    const fs::path episode_dir = output_dir_ / "episodes" /
      ("episode_" + std::to_string(episode_index));
    fs::create_directories(episode_dir / "frames");
    std::ofstream steps(episode_dir / "steps.jsonl");
    if (!steps) {
      throw std::runtime_error("cannot write raw VLA steps");
    }
    const std::string instruction = instruction_for(color, episode_index);
    std::vector<bool> visited(viewpoints.size(), false);
    int last_action = -1;
    int frame_index = 0;
    std::size_t current = randomizer() % viewpoints.size();
    bool finished = false;
    while (!finished && rclcpp::ok()) {
      set_robot_pose(viewpoints[current], viewpoints[current].yaw);
      visited[current] = true;
      auto state = build_vla_policy_state(
        viewpoints[current].x, viewpoints[current].y, viewpoints[current].yaw,
        viewpoints, visited, last_action, 0.0);
      auto frame = capture_frame();
      const std::size_t target_label_pixels = target_present ? count_semantic_label(
        frame, static_cast<std::uint8_t>(approach_action(color) - 4)) : 0;
      const bool target_visible = target_label_pixels > 0;
      const int first_action = target_visible ? approach_action(color) : static_cast<int>(VLAAction::SCAN_LEFT);
      save_step(episode_dir, frame_index, frame, instruction, state, first_action,
        target_label_pixels, false, steps);
      if (target_visible) {
        break;
      }

      set_robot_pose(viewpoints[current], viewpoints[current].yaw + kScanOffset);
      state = build_vla_policy_state(
        viewpoints[current].x, viewpoints[current].y, viewpoints[current].yaw + kScanOffset,
        viewpoints, visited, first_action, 0.5);
      frame = capture_frame();
      const std::size_t left_target_label_pixels = target_present ? count_semantic_label(
        frame, static_cast<std::uint8_t>(approach_action(color) - 4)) : 0;
      const bool left_target_visible = left_target_label_pixels > 0;
      const int second_action = left_target_visible ? approach_action(color) : static_cast<int>(VLAAction::SCAN_RIGHT);
      save_step(episode_dir, frame_index, frame, instruction, state, second_action,
        left_target_label_pixels, false, steps);
      if (left_target_visible) {
        break;
      }

      set_robot_pose(viewpoints[current], viewpoints[current].yaw - kScanOffset);
      state = build_vla_policy_state(
        viewpoints[current].x, viewpoints[current].y, viewpoints[current].yaw - kScanOffset,
        viewpoints, visited, second_action, 1.0);
      frame = capture_frame();
      const std::size_t right_target_label_pixels = target_present ? count_semantic_label(
        frame, static_cast<std::uint8_t>(approach_action(color) - 4)) : 0;
      const bool right_target_visible = right_target_label_pixels > 0;
      if (right_target_visible) {
        save_step(
          episode_dir, frame_index, frame, instruction, state, approach_action(color),
          right_target_label_pixels, false, steps);
        break;
      }
      const int next_action = select_candidate_action(state);
      const bool coverage_complete = next_action < 0;
      if (coverage_complete && target_present) {
        // A placed target is guaranteed to have a searchable view; emitting a false negative is unrecoverable.
        RCLCPP_ERROR(
          get_logger(),
          "Oracle target was never visible: seed=%llu color=%s target_viewpoint=%zu",
          static_cast<unsigned long long>(seed), color.c_str(), target_viewpoint);
        throw std::runtime_error("target-present Oracle episode completed without a visible target");
      }
      save_step(
        episode_dir, frame_index, frame, instruction, state,
        coverage_complete ? static_cast<int>(VLAAction::REPORT_NOT_FOUND) : next_action,
        right_target_label_pixels, coverage_complete, steps);
      last_action = coverage_complete ? static_cast<int>(VLAAction::REPORT_NOT_FOUND) : next_action;
      if (coverage_complete) {
        finished = true;
      } else {
        current = *state.candidate_indices.at(static_cast<std::size_t>(next_action));
      }
    }
    write_text_file(episode_dir / "episode.json", json{
      {"episode_id", episode_index}, {"seed", seed}, {"split", split},
      {"target_color", color}, {"target_present", target_present}
    }.dump(2) + "\n");
    RCLCPP_INFO(get_logger(), "Recorded VLA oracle episode %d/%d", episode_index + 1, episode_count_);
  }

  static std::optional<std::size_t> nearest_unvisited_viewpoint(
    const VLAViewpoint & robot, const std::vector<VLAViewpoint> & viewpoints,
    const std::vector<bool> & visited)
  {
    std::optional<std::size_t> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < viewpoints.size(); ++index) {
      if (visited[index]) {
        continue;
      }
      const double distance = std::hypot(viewpoints[index].x - robot.x, viewpoints[index].y - robot.y);
      if (distance < best_distance) {
        best = index;
        best_distance = distance;
      }
    }
    return best;
  }

  static ContinuousVLAAction local_goal_towards(
    const VLAViewpoint & robot, const VLAViewpoint & target, const double requested_distance)
  {
    // Keep Oracle labels inside the runtime safety envelope so conversion cannot train unsafe output.
    const double map_dx = target.x - robot.x;
    const double map_dy = target.y - robot.y;
    const double distance = std::hypot(map_dx, map_dy);
    const double travel = distance < 1e-6 ? 0.0 : std::min(
      {std::max(0.0, requested_distance), distance, kMaxLocalGoalDistanceM});
    const double scaled_dx = distance < 1e-6 ? 0.0 : map_dx * travel / distance;
    const double scaled_dy = distance < 1e-6 ? 0.0 : map_dy * travel / distance;
    const double yaw_error = distance < 1e-6 ? 0.0 : normalize_angle(std::atan2(map_dy, map_dx) - robot.yaw);
    return ContinuousVLAAction{
      std::cos(robot.yaw) * scaled_dx + std::sin(robot.yaw) * scaled_dy,
      -std::sin(robot.yaw) * scaled_dx + std::cos(robot.yaw) * scaled_dy,
      std::clamp(yaw_error, -kMaxLocalGoalYawRad, kMaxLocalGoalYawRad)};
  }

  void save_continuous_step(
    const fs::path & episode_dir, int & frame_index, const CapturedFrame & capture,
    const std::string & stage_instruction, const ContinuousVLAState & state,
    const ContinuousVLAAction & action, const std::string & stage,
    const std::size_t target_label_pixels, const VLAViewpoint & robot,
    const std::optional<VLAViewpoint> & target, std::ofstream & steps)
  {
    const std::string file_name = "frames/" + std::to_string(frame_index++) + ".jpg";
    const auto image = cv_bridge::toCvCopy(capture.image, "bgr8");
    if (!cv::imwrite((episode_dir / file_name).string(), image->image)) {
      throw std::runtime_error("failed to write continuous Oracle RGB frame");
    }
    json oracle{
      {"target_visible", target_label_pixels > 0},
      {"target_label_pixels", target_label_pixels},
      {"stage", stage},
      {"robot_pose", {robot.x, robot.y, robot.yaw}}};
    if (target.has_value()) {
      oracle["target_pose"] = {target->x, target->y};
    }
    steps << json{
      {"image", file_name},
      {"instruction", stage_instruction},
      {"robot_state", state.values},
      {"action", {action.delta_x_m, action.delta_y_m, action.delta_yaw_rad}},
      {"oracle", oracle}
    }.dump() << '\n';
  }

  void generate_continuous_episode(const int episode_index)
  {
    remove_balls();
    const std::uint64_t seed = static_cast<std::uint64_t>(master_seed_) * 1000003ULL + episode_index;
    std::mt19937_64 randomizer(seed);
    const std::string color = kColors.at(randomizer() % kColors.size());
    const bool target_present = std::uniform_real_distribution<double>(0.0, 1.0)(randomizer) >=
      target_absent_ratio_;
    const std::string split = split_for_episode(episode_index, episode_count_);
    std::vector<VLAViewpoint> viewpoints;
    viewpoints.reserve(viewpoints_.size() / 3);
    for (std::size_t index = 0; index < viewpoints_.size() / 3; ++index) {
      viewpoints.push_back(VLAViewpoint{
        index, viewpoints_[index * 3], viewpoints_[index * 3 + 1], viewpoints_[index * 3 + 2]});
    }
    const std::size_t target_viewpoint = randomizer() % viewpoints.size();
    const double target_yaw = viewpoints[target_viewpoint].yaw + 0.20;
    const VLAViewpoint target_pose{
      0U, viewpoints[target_viewpoint].x + 0.90 * std::cos(target_yaw),
      viewpoints[target_viewpoint].y + 0.90 * std::sin(target_yaw), target_yaw};
    if (target_present) {
      spawn_ball("oracle_target_" + std::to_string(seed), color, viewpoints[target_viewpoint], 0, 0.90);
    }
    const int distractor_count = 1 + static_cast<int>(randomizer() % 3);
    const auto target_color_iterator = std::find(kColors.begin(), kColors.end(), color);
    const std::size_t target_color_index = static_cast<std::size_t>(
      std::distance(kColors.begin(), target_color_iterator));
    for (int ordinal = 0; ordinal < distractor_count; ++ordinal) {
      const std::string distractor_color = target_present ?
        kColors.at(randomizer() % kColors.size()) :
        kColors.at((target_color_index + static_cast<std::size_t>(ordinal) + 1U) % kColors.size());
      const std::size_t position = (target_viewpoint + 1 + ordinal * 3) % viewpoints.size();
      spawn_ball(
        "oracle_distractor_" + std::to_string(seed) + "_" + std::to_string(ordinal),
        distractor_color, viewpoints[position], static_cast<std::size_t>(ordinal + 1), 0.75);
    }

    const fs::path episode_dir = output_dir_ / "episodes" /
      ("episode_" + std::to_string(episode_index));
    fs::create_directories(episode_dir / "frames");
    std::ofstream steps(episode_dir / "steps.jsonl");
    if (!steps) {
      throw std::runtime_error("cannot write continuous raw VLA steps");
    }

    std::vector<bool> visited(viewpoints.size(), false);
    ContinuousVLAAction previous_action{0.0, 0.0, 0.0};
    std::size_t current = randomizer() % viewpoints.size();
    int frame_index = 0;
    bool target_reached = false;
    while (rclcpp::ok() && !target_reached) {
      visited[current] = true;
      const auto next = nearest_unvisited_viewpoint(viewpoints[current], viewpoints, visited);
      for (const double scan_offset : std::array<double, 3>{0.0, kScanOffset, -kScanOffset}) {
        VLAViewpoint robot = viewpoints[current];
        robot.yaw += scan_offset;
        set_robot_pose(robot, robot.yaw);
        const auto frame = capture_frame();
        const std::size_t target_label_pixels = target_present ? count_semantic_label(
          frame, static_cast<std::uint8_t>(approach_action(color) - 4)) : 0;
        const bool target_visible = target_label_pixels > 0;
        if (!target_visible && !next.has_value()) {
          continue;
        }
        const std::optional<VLAViewpoint> state_target = target_visible ?
          std::optional<VLAViewpoint>(target_pose) : std::nullopt;
        const ContinuousVLAAction action = target_visible ?
          local_goal_towards(robot, target_pose, std::max(0.0, std::hypot(
            target_pose.x - robot.x, target_pose.y - robot.y) - 0.80)) :
          local_goal_towards(robot, viewpoints[*next], std::hypot(
            viewpoints[*next].x - robot.x, viewpoints[*next].y - robot.y));
        const auto state = build_continuous_vla_policy_state(
          robot.x, robot.y, robot.yaw, viewpoints, visited, state_target,
          target_visible ? 0.9 : 0.0, 0.0, 0.0, previous_action,
          target_visible ? 7.0 / 15.0 : 0.0, true);
        save_continuous_step(
          episode_dir, frame_index, frame, stage_instruction_for(color, target_visible), state, action,
          target_visible ? "approach" : "search", target_label_pixels, robot, state_target, steps);
        previous_action = action;
        if (target_visible) {
          target_reached = true;
          break;
        }
      }
      if (target_reached || !next.has_value()) {
        break;
      }
      current = *next;
    }
    if (target_present && !target_reached) {
      throw std::runtime_error("target-present continuous Oracle episode completed without a visible target");
    }
    write_text_file(episode_dir / "episode.json", json{
      {"episode_id", episode_index}, {"seed", seed}, {"split", split},
      {"target_color", color}, {"target_present", target_present},
      {"coverage_complete", !target_present},
      {"policy_interface", "continuous_local_goal"}
    }.dump(2) + "\n");
    RCLCPP_INFO(
      get_logger(), "Recorded continuous VLA oracle episode %d/%d", episode_index + 1, episode_count_);
  }

  void write_generation_report() const
  {
    json report{
      {"episodes", episode_count_}, {"target_absent_ratio", target_absent_ratio_},
      {"seed", master_seed_}, {"policy_interface", policy_interface_},
      {"status", "raw episodes recorded"}
    };
    write_text_file(output_dir_ / "generation_report.json", report.dump(2) + "\n");
  }

  fs::path output_dir_;
  fs::path approval_file_;
  int episode_count_;
  std::int64_t master_seed_;
  double target_absent_ratio_;
  std::string policy_interface_;
  bool resume_;
  int settle_frames_;
  double capture_timeout_;
  std::string world_name_;
  std::string robot_name_;
  std::string label_topic_;
  std::vector<double> viewpoints_;
  gz::transport::Node gz_node_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  Image::ConstSharedPtr latest_image_;
  std::vector<std::uint8_t> latest_labels_;
  std::uint32_t label_width_{0};
  std::uint32_t label_height_{0};
  std::uint32_t label_step_{0};
  std::size_t image_sequence_{0};
  std::size_t label_sequence_{0};
  std::vector<std::string> spawned_entities_;
};

}  // namespace turtlebot3_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::exception_ptr run_error;
  try {
    auto node = std::make_shared<turtlebot3_gazebo::VLAOracleDatasetGenerator>();
    std::thread runner([&node, &run_error]() {
      try {
        node->run();
      } catch (...) {
        run_error = std::current_exception();
      }
      rclcpp::shutdown();
    });
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    runner.join();
    node->stop_transport();
    if (run_error != nullptr) {
      std::rethrow_exception(run_error);
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vla_oracle_dataset_generator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
