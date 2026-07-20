// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <turtlebot3_embodied_interfaces/action/find_object.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "turtlebot3_embodied_navigation/detection_history.hpp"
#include "turtlebot3_embodied_navigation/instruction_parser.hpp"

namespace turtlebot3_embodied_navigation
{

class FindObjectServer : public rclcpp::Node
{
public:
  using FindObject = turtlebot3_embodied_interfaces::action::FindObject;
  using FindGoalHandle = rclcpp_action::ServerGoalHandle<FindObject>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FindObjectServer()
  : Node("find_object_server"),
    detection_history_(15, 7),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    allowed_classes_ = declare_parameter<std::vector<std::string>>(
      "allowed_classes", {"cup", "bottle", "backpack", "ball", "box"});
    allowed_colors_ = declare_parameter<std::vector<std::string>>(
      "allowed_colors", {"red", "green", "blue", "yellow", "unknown"});
    allowed_rooms_ = declare_parameter<std::vector<std::string>>(
      "allowed_rooms", {"kitchen", "living_room", "bedroom", "unknown"});
    viewpoints_ = declare_parameter<std::vector<double>>(
      "search_viewpoints", std::vector<double>{});
    viewpoint_rooms_ = declare_parameter<std::vector<std::string>>(
      "viewpoint_rooms", std::vector<std::string>{});
    approach_distance_ = declare_parameter("approach_distance", 0.8);
    task_timeout_ = declare_parameter("task_timeout", 300.0);
    navigation_timeout_ = declare_parameter("navigation_timeout", 90.0);
    verification_timeout_ = declare_parameter("verification_timeout", 5.0);
    maximum_candidates_ = declare_parameter("maximum_candidates", 20);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");

    if (viewpoints_.size() % 3 != 0) {
      throw std::invalid_argument("search_viewpoints must contain x, y, yaw triples");
    }
    if (!viewpoint_rooms_.empty() && viewpoint_rooms_.size() != viewpoints_.size() / 3) {
      throw std::invalid_argument("viewpoint_rooms must match search_viewpoints");
    }
    if (approach_distance_ <= 0.0 || task_timeout_ <= 0.0 || navigation_timeout_ <= 0.0 ||
      verification_timeout_ <= 0.0 || maximum_candidates_ <= 0)
    {
      throw std::invalid_argument("task limits and timeouts must be positive");
    }

    detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "/embodied/detections", rclcpp::SensorDataQoS(),
      std::bind(&FindObjectServer::on_detections, this, std::placeholders::_1));
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    action_server_ = rclcpp_action::create_server<FindObject>(
      this, "find_object",
      std::bind(&FindObjectServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FindObjectServer::on_cancel, this, std::placeholders::_1),
      std::bind(&FindObjectServer::on_accepted, this, std::placeholders::_1));
  }

private:
  enum class NavigationOutcome {SUCCEEDED, FAILED, CANCELED, TARGET_FOUND, TIMED_OUT};

  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const FindObject::Goal> goal)
  {
    if (goal->instruction.empty() || is_executing_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<FindGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<FindGoalHandle> goal_handle)
  {
    std::thread(&FindObjectServer::execute, this, goal_handle).detach();
  }

  static std::string environment(const char * name)
  {
    const char * value = std::getenv(name);
    return value == nullptr ? "" : value;
  }

  void execute(const std::shared_ptr<FindGoalHandle> goal_handle)
  {
    struct ExecutionGuard
    {
      std::atomic_bool & flag;
      ~ExecutionGuard() {flag.store(false);}
    } guard{is_executing_};

    const auto started = std::chrono::steady_clock::now();
    TargetSpec target;
    publish_feedback(goal_handle, FindObject::Feedback::PARSING, "parsing", 0, started);
    const std::string api_base = environment("VLM_API_BASE");
    const std::string model = environment("VLM_MODEL");
    const std::string api_key = environment("VLM_API_KEY");
    if (api_base.empty() || model.empty() || api_key.empty()) {
      finish(
        goal_handle, false, FindObject::Result::LANGUAGE_API_ERROR,
        "VLM_API_BASE, VLM_MODEL and VLM_API_KEY are required", target, started);
      return;
    }
    try {
      OpenAiInstructionParser parser(
        api_base, model, api_key, allowed_classes_, allowed_colors_, allowed_rooms_);
      target = parser.parse(goal_handle->get_goal()->instruction);
    } catch (const std::invalid_argument & error) {
      finish(goal_handle, false, FindObject::Result::INVALID_INSTRUCTION, error.what(), target, started);
      return;
    } catch (const std::exception & error) {
      finish(goal_handle, false, FindObject::Result::LANGUAGE_API_ERROR, error.what(), target, started);
      return;
    }

    reset_detection(target.object_class + ":" + target.color);
    const auto candidates = ordered_candidate_indices(target.room);
    if (candidates.empty()) {
      finish(
        goal_handle, false, FindObject::Result::TARGET_NOT_FOUND,
        "no search viewpoints are configured for the requested room", target, started);
      return;
    }

    std::size_t visited = 0;
    for (const std::size_t index : candidates) {
      if (expired(started)) {
        finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
        return;
      }
      publish_feedback(
        goal_handle, FindObject::Feedback::SEARCHING_VIEWPOINTS, "searching_viewpoints",
        visited, started, target.room, candidates.size());
      const auto outcome = navigate_to(goal_handle, viewpoint(index), started, true);
      ++visited;
      if (outcome == NavigationOutcome::CANCELED) {
        finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
        return;
      }
      if (outcome == NavigationOutcome::TIMED_OUT) {
        finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
        return;
      }
      if (outcome == NavigationOutcome::TARGET_FOUND || detection_confirmed()) {
        approach_and_verify(goal_handle, target, started, visited, candidates.size());
        return;
      }
      if (outcome == NavigationOutcome::FAILED) {
        RCLCPP_WARN(get_logger(), "Skipping unreachable search viewpoint %zu", index);
      }
    }
    finish(
      goal_handle, false, FindObject::Result::TARGET_NOT_FOUND,
      "target was not confirmed at any configured viewpoint", target, started);
  }

  void approach_and_verify(
    const std::shared_ptr<FindGoalHandle> & goal_handle, const TargetSpec & target,
    const std::chrono::steady_clock::time_point & started, const std::size_t visited,
    const std::size_t total)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    float confidence = 0.0F;
    if (!target_pose_in_map(target_pose, confidence)) {
      finish(
        goal_handle, false, FindObject::Result::PERCEPTION_TIMEOUT,
        "confirmed detection could not be transformed to map", target, started);
      return;
    }

    geometry_msgs::msg::PoseStamped approach_pose;
    try {
      approach_pose = make_approach_pose(target_pose);
    } catch (const tf2::TransformException & error) {
      finish(goal_handle, false, FindObject::Result::NAVIGATION_FAILED, error.what(), target, started);
      return;
    }
    publish_feedback(
      goal_handle, FindObject::Feedback::APPROACHING, "approaching", visited, started,
      target.room, total);
    const auto outcome = navigate_to(goal_handle, approach_pose, started, false);
    if (outcome != NavigationOutcome::SUCCEEDED) {
      uint8_t code = FindObject::Result::NAVIGATION_FAILED;
      if (outcome == NavigationOutcome::CANCELED) {
        code = FindObject::Result::CANCELED;
      } else if (outcome == NavigationOutcome::TIMED_OUT) {
        code = FindObject::Result::TIMEOUT;
      }
      finish(goal_handle, false, code, "failed to reach the target approach pose", target, started);
      return;
    }

    reset_detection(target.object_class + ":" + target.color);
    publish_feedback(
      goal_handle, FindObject::Feedback::VERIFYING, "verifying", visited, started,
      target.room, total);
    const auto verify_started = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - verify_started).count() <
      verification_timeout_)
    {
      if (goal_handle->is_canceling()) {
        finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
        return;
      }
      if (detection_confirmed()) {
        target_pose_in_map(target_pose, confidence);
        finish(
          goal_handle, true, FindObject::Result::NONE, "target found and verified", target,
          started, target_pose, confidence);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    finish(
      goal_handle, false, FindObject::Result::PERCEPTION_TIMEOUT,
      "target was not confirmed after approach", target, started, target_pose, confidence);
  }

  NavigationOutcome navigate_to(
    const std::shared_ptr<FindGoalHandle> & task, const geometry_msgs::msg::PoseStamped & pose,
    const std::chrono::steady_clock::time_point & task_started, const bool stop_on_detection)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      return NavigationOutcome::FAILED;
    }
    NavigateToPose::Goal goal;
    goal.pose = pose;
    const auto goal_future = nav_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return NavigationOutcome::FAILED;
    }
    const auto nav_goal = goal_future.get();
    if (!nav_goal) {
      return NavigationOutcome::FAILED;
    }

    const auto result_future = nav_client_->async_get_result(nav_goal);
    const auto navigation_started = std::chrono::steady_clock::now();
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (task->is_canceling()) {
        cancel_navigation(nav_goal);
        return NavigationOutcome::CANCELED;
      }
      if (expired(task_started)) {
        cancel_navigation(nav_goal);
        return NavigationOutcome::TIMED_OUT;
      }
      if (stop_on_detection && detection_confirmed()) {
        cancel_navigation(nav_goal);
        return NavigationOutcome::TARGET_FOUND;
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - navigation_started).count() >
        navigation_timeout_)
      {
        cancel_navigation(nav_goal);
        return NavigationOutcome::FAILED;
      }
    }
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED ?
           NavigationOutcome::SUCCEEDED : NavigationOutcome::FAILED;
  }

  void cancel_navigation(const NavGoalHandle::SharedPtr & goal)
  {
    const auto cancel_future = nav_client_->async_cancel_goal(goal);
    if (cancel_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Nav2 did not acknowledge goal cancellation within 2 seconds");
    }
  }

  std::vector<std::size_t> ordered_candidate_indices(const std::string & room) const
  {
    std::vector<std::size_t> preferred;
    std::vector<std::size_t> fallback;
    const std::size_t count = std::min(
      viewpoints_.size() / 3, static_cast<std::size_t>(maximum_candidates_));
    for (std::size_t index = 0; index < count; ++index) {
      if (!viewpoint_rooms_.empty() && viewpoint_rooms_[index] == room) {
        preferred.push_back(index);
      } else {
        fallback.push_back(index);
      }
    }
    preferred.insert(preferred.end(), fallback.begin(), fallback.end());
    return preferred;
  }

  geometry_msgs::msg::PoseStamped viewpoint(const std::size_t index) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = viewpoints_[index * 3];
    pose.pose.position.y = viewpoints_[index * 3 + 1];
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, viewpoints_[index * 3 + 2]);
    pose.pose.orientation = tf2::toMsg(orientation);
    return pose;
  }

  geometry_msgs::msg::PoseStamped make_approach_pose(
    const geometry_msgs::msg::PoseStamped & target) const
  {
    const auto robot_transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    const double dx = robot_transform.transform.translation.x - target.pose.position.x;
    const double dy = robot_transform.transform.translation.y - target.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1e-3) {
      throw tf2::TransformException("robot and target positions are indistinguishable");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();
    const double offset = std::min(approach_distance_, distance);
    pose.pose.position.x = target.pose.position.x + dx * offset / distance;
    pose.pose.position.y = target.pose.position.y + dy * offset / distance;
    tf2::Quaternion orientation;
    orientation.setRPY(
      0.0, 0.0,
      std::atan2(target.pose.position.y - pose.pose.position.y,
      target.pose.position.x - pose.pose.position.x));
    pose.pose.orientation = tf2::toMsg(orientation);
    return pose;
  }

  void on_detections(const vision_msgs::msg::Detection3DArray::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (target_label_.empty()) {
      return;
    }
    const vision_msgs::msg::Detection3D * best = nullptr;
    float best_score = 0.0F;
    for (const auto & detection : message->detections) {
      if (!detection.results.empty() && detection.results.front().hypothesis.class_id == target_label_ &&
        detection.results.front().hypothesis.score > best_score)
      {
        best = &detection;
        best_score = detection.results.front().hypothesis.score;
      }
    }
    detection_history_.add(best != nullptr);
    if (best != nullptr) {
      latest_detection_.header = best->header;
      latest_detection_.pose = best->bbox.center;
      best_confidence_ = std::max(best_confidence_, best_score);
    }
  }

  void reset_detection(const std::string & target_label)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    target_label_ = target_label;
    detection_history_.reset();
    latest_detection_ = geometry_msgs::msg::PoseStamped();
    best_confidence_ = 0.0F;
  }

  bool detection_confirmed()
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    return detection_history_.confirmed();
  }

  bool target_pose_in_map(geometry_msgs::msg::PoseStamped & target, float & confidence)
  {
    geometry_msgs::msg::PoseStamped source;
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      source = latest_detection_;
      confidence = best_confidence_;
    }
    if (source.header.frame_id.empty()) {
      return false;
    }
    try {
      target = tf_buffer_.transform(source, map_frame_, tf2::durationFromSec(0.5));
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "Detection transform failed: %s", error.what());
      return false;
    }
  }

  bool expired(const std::chrono::steady_clock::time_point & started) const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >
           task_timeout_;
  }

  builtin_interfaces::msg::Duration elapsed(
    const std::chrono::steady_clock::time_point & started) const
  {
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - started).count();
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return duration;
  }

  void publish_feedback(
    const std::shared_ptr<FindGoalHandle> & goal, const uint8_t phase,
    const std::string & phase_name, const std::size_t visited,
    const std::chrono::steady_clock::time_point & started,
    const std::string & region = "", const std::size_t total = 0)
  {
    auto feedback = std::make_shared<FindObject::Feedback>();
    feedback->phase = phase;
    feedback->phase_name = phase_name;
    feedback->current_region = region;
    feedback->visited_candidates = visited;
    feedback->total_candidates = total;
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      feedback->best_confidence = best_confidence_;
    }
    feedback->elapsed = elapsed(started);
    goal->publish_feedback(feedback);
  }

  void finish(
    const std::shared_ptr<FindGoalHandle> & goal, const bool success, const uint8_t failure_code,
    const std::string & message, const TargetSpec & target,
    const std::chrono::steady_clock::time_point & started,
    const geometry_msgs::msg::PoseStamped & pose = geometry_msgs::msg::PoseStamped(),
    const float confidence = 0.0F)
  {
    auto result = std::make_shared<FindObject::Result>();
    result->success = success;
    result->failure_code = failure_code;
    result->message = message;
    result->target_class = target.object_class;
    result->target_color = target.color;
    result->target_room = target.room;
    result->target_pose = pose;
    result->final_confidence = confidence;
    result->elapsed = elapsed(started);
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      target_label_.clear();
    }
    if (goal->is_canceling() || failure_code == FindObject::Result::CANCELED) {
      goal->canceled(result);
    } else if (success) {
      goal->succeed(result);
    } else {
      goal->abort(result);
    }
  }

  std::vector<std::string> allowed_classes_;
  std::vector<std::string> allowed_colors_;
  std::vector<std::string> allowed_rooms_;
  std::vector<double> viewpoints_;
  std::vector<std::string> viewpoint_rooms_;
  double approach_distance_;
  double task_timeout_;
  double navigation_timeout_;
  double verification_timeout_;
  int maximum_candidates_;
  std::string map_frame_;
  std::string robot_frame_;
  std::atomic_bool is_executing_{false};

  std::mutex detection_mutex_;
  DetectionHistory detection_history_;
  std::string target_label_;
  geometry_msgs::msg::PoseStamped latest_detection_;
  float best_confidence_{0.0F};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Server<FindObject>::SharedPtr action_server_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
};

}  // namespace turtlebot3_embodied_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<turtlebot3_embodied_navigation::FindObjectServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("find_object_server"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
