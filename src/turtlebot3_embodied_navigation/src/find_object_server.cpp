// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
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
#include "turtlebot3_embodied_navigation/skill_policy_client.hpp"
#include "turtlebot3_embodied_navigation/vla_policy_state.hpp"

namespace turtlebot3_embodied_navigation
{

class FindObjectServer : public rclcpp::Node
{
public:
  using FindObject = turtlebot3_embodied_interfaces::action::FindObject;
  using FindGoalHandle = rclcpp_action::ServerGoalHandle<FindObject>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Spin = nav2_msgs::action::Spin;
  using SpinGoalHandle = rclcpp_action::ClientGoalHandle<Spin>;

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
    maximum_policy_decisions_ = declare_parameter("maximum_policy_decisions", 30);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    policy_mode_ = declare_parameter<std::string>("policy_mode", "disabled");
    policy_endpoint_ = declare_parameter<std::string>(
      "policy_endpoint", "http://127.0.0.1:8089/select_action");
    policy_interface_ = declare_parameter<std::string>("policy_interface", "discrete_skill");
    continuous_policy_endpoint_ = declare_parameter<std::string>(
      "continuous_policy_endpoint", "http://127.0.0.1:8089/predict_local_goal");
    policy_timeout_milliseconds_ = declare_parameter("policy_timeout_milliseconds", 1000);
    policy_max_prototype_distance_ = declare_parameter("policy_max_prototype_distance", 1.0);
    policy_min_prototype_gap_ = declare_parameter("policy_min_prototype_gap", 0.1);
    policy_image_max_age_ = declare_parameter("policy_image_max_age", 1.0);
    detection_max_age_ = declare_parameter("detection_max_age", 1.0);
    maximum_planner_stages_ = declare_parameter("maximum_planner_stages", 8);
    maximum_local_goals_per_stage_ = declare_parameter("maximum_local_goals_per_stage", 30);
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");

    if (viewpoints_.size() % 3 != 0) {
      throw std::invalid_argument("search_viewpoints must contain x, y, yaw triples");
    }
    if (!viewpoint_rooms_.empty() && viewpoint_rooms_.size() != viewpoints_.size() / 3) {
      throw std::invalid_argument("viewpoint_rooms must match search_viewpoints");
    }
    if (approach_distance_ <= 0.0 || task_timeout_ <= 0.0 || navigation_timeout_ <= 0.0 ||
      verification_timeout_ <= 0.0 || maximum_candidates_ <= 0 || maximum_policy_decisions_ <= 0 ||
      policy_timeout_milliseconds_ <= 0 || policy_max_prototype_distance_ < 0.0 ||
      policy_min_prototype_gap_ < 0.0 || policy_image_max_age_ <= 0.0 ||
      detection_max_age_ <= 0.0 || maximum_planner_stages_ <= 0 ||
      maximum_local_goals_per_stage_ <= 0)
    {
      throw std::invalid_argument("task limits and timeouts must be positive");
    }
    if (policy_mode_ != "disabled" && policy_mode_ != "shadow" && policy_mode_ != "active") {
      throw std::invalid_argument("policy_mode must be disabled, shadow or active");
    }
    if (policy_interface_ != "discrete_skill" && policy_interface_ != "continuous_local_goal") {
      throw std::invalid_argument("policy_interface must be discrete_skill or continuous_local_goal");
    }
    if (policy_mode_ != "disabled") {
      if (policy_interface_ == "discrete_skill") {
        policy_client_.emplace(policy_endpoint_, policy_timeout_milliseconds_);
      } else {
        continuous_policy_client_.emplace(
          continuous_policy_endpoint_, policy_timeout_milliseconds_);
      }
    }

    detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "/embodied/detections", rclcpp::SensorDataQoS(),
      std::bind(&FindObjectServer::on_detections, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FindObjectServer::on_image, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&FindObjectServer::on_odom, this, std::placeholders::_1));
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");
    action_server_ = rclcpp_action::create_server<FindObject>(
      this, "find_object",
      std::bind(&FindObjectServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FindObjectServer::on_cancel, this, std::placeholders::_1),
      std::bind(&FindObjectServer::on_accepted, this, std::placeholders::_1));
  }

private:
  enum class NavigationOutcome {SUCCEEDED, FAILED, CANCELED, TARGET_FOUND, TIMED_OUT};
  enum class ContinuousStageOutcome {
    TARGET_DETECTED,
    COVERAGE_COMPLETE,
    APPROACH_REACHED,
    TARGET_VERIFIED,
    DETECTION_LOST,
    CANCELED,
    TIMED_OUT,
    FAILED,
  };

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
    std::optional<OpenAiInstructionParser> parser;
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
      parser.emplace(
        api_base, model, api_key, allowed_classes_, allowed_colors_, allowed_rooms_);
      target = parser->parse(goal_handle->get_goal()->instruction);
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

    if (policy_mode_ == "active") {
      if (policy_interface_ == "continuous_local_goal") {
        execute_continuous_policy(goal_handle, *parser, target, candidates, started);
        return;
      }
      execute_active_policy(goal_handle, target, candidates, started);
      return;
    }

    std::size_t visited = 0;
    std::vector<bool> visited_candidates(candidates.size(), false);
    for (const std::size_t index : candidates) {
      if (expired(started)) {
        finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
        return;
      }
      publish_feedback(
        goal_handle, FindObject::Feedback::SEARCHING_VIEWPOINTS, "searching_viewpoints",
        visited, started, target.room, candidates.size());
      if (policy_mode_ == "shadow") {
        if (policy_interface_ == "continuous_local_goal") {
          log_continuous_shadow_decision(
            "search for " + target.color + " " + target.object_class, candidates,
            visited_candidates, std::nullopt, std::nullopt, true);
        } else {
          log_shadow_decision(
            goal_handle->get_goal()->instruction, candidates, visited_candidates, -1, 1.0);
        }
      }
      const auto outcome = navigate_to(goal_handle, viewpoint(index), started, true);
      const auto candidate_position = std::find(candidates.begin(), candidates.end(), index);
      if (candidate_position != candidates.end()) {
        visited_candidates[static_cast<std::size_t>(candidate_position - candidates.begin())] = true;
      }
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

  void execute_active_policy(
    const std::shared_ptr<FindGoalHandle> & goal_handle, const TargetSpec & target,
    const std::vector<std::size_t> & candidates,
    const std::chrono::steady_clock::time_point & started)
  {
    std::vector<bool> visited(candidates.size(), false);
    std::optional<std::size_t> current_viewpoint;
    int last_action_id = -1;
    double scan_progress = 1.0;

    for (int decision_count = 0; decision_count < maximum_policy_decisions_; ++decision_count) {
      if (goal_handle->is_canceling()) {
        finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
        return;
      }
      if (expired(started)) {
        finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
        return;
      }

      VLAState state;
      VLAActionDecision decision;
      try {
        state = current_vla_state(candidates, visited, last_action_id, scan_progress);
        decision = request_policy_decision(goal_handle->get_goal()->instruction, state);
      } catch (const std::exception & error) {
        finish(
          goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
          std::string("VLA policy request failed: ") + error.what(), target, started);
        return;
      }
      if (!policy_decision_accepted(decision)) {
        finish(
          goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
          "VLA policy decision failed calibrated prototype gate", target, started);
        return;
      }
      RCLCPP_INFO(
        get_logger(), "VLA action=%s distance=%.3f gap=%.3f model=%s",
        vla_action_name(decision.action).c_str(), decision.prototype_distance,
        decision.prototype_gap, decision.model_version.c_str());
      last_action_id = static_cast<int>(decision.action);

      if (is_vla_candidate_action(decision.action)) {
        if (scan_progress < 1.0) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "VLA selected a new viewpoint before completing the current scan", target, started);
          return;
        }
        const auto slot = vla_candidate_slot(decision.action);
        if (!state.candidate_indices[slot].has_value()) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "VLA selected an invalid candidate slot", target, started);
          return;
        }
        const std::size_t viewpoint_index = *state.candidate_indices[slot];
        publish_feedback(
          goal_handle, FindObject::Feedback::SEARCHING_VIEWPOINTS, "searching_viewpoints",
          count_visited(visited), started, target.room, candidates.size());
        const auto outcome = navigate_to(goal_handle, viewpoint(viewpoint_index), started, true);
        if (outcome == NavigationOutcome::CANCELED) {
          finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
          return;
        }
        if (outcome == NavigationOutcome::TIMED_OUT) {
          finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
          return;
        }
        if (outcome == NavigationOutcome::TARGET_FOUND) {
          current_viewpoint.reset();
          scan_progress = 1.0;
          continue;
        }
        const auto position = candidate_position(candidates, viewpoint_index);
        if (position.has_value()) {
          visited[*position] = true;
        }
        if (outcome == NavigationOutcome::FAILED) {
          RCLCPP_WARN(get_logger(), "VLA candidate %zu was unreachable", viewpoint_index);
          current_viewpoint.reset();
          scan_progress = 1.0;
          continue;
        }
        current_viewpoint = viewpoint_index;
        scan_progress = 0.0;
        continue;
      }

      if (decision.action == VLAAction::SCAN_LEFT || decision.action == VLAAction::SCAN_RIGHT) {
        const bool is_left = decision.action == VLAAction::SCAN_LEFT;
        const bool expected_phase = is_left ? scan_progress == 0.0 : scan_progress == 0.5;
        if (!current_viewpoint.has_value() || !expected_phase) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "VLA selected an invalid scan transition", target, started);
          return;
        }
        const auto outcome = spin_for_scan(goal_handle, *current_viewpoint, is_left, started);
        if (outcome == NavigationOutcome::CANCELED) {
          finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
          return;
        }
        if (outcome == NavigationOutcome::TIMED_OUT) {
          finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
          return;
        }
        if (outcome != NavigationOutcome::SUCCEEDED) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "Nav2 failed while executing VLA scan", target, started);
          return;
        }
        scan_progress = is_left ? 0.5 : 1.0;
        continue;
      }

      if (const auto color = vla_approach_color(decision.action); color.has_value()) {
        if (*color != target.color) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "VLA approach color conflicts with the independent language contract", target, started);
          return;
        }
        if (!detection_confirmed()) {
          finish(
            goal_handle, false, FindObject::Result::PERCEPTION_TIMEOUT,
            "VLA requested approach without a fresh stable target detection", target, started);
          return;
        }
        approach_and_verify(
          goal_handle, target, started, count_visited(visited), candidates.size());
        return;
      }

      if (decision.action == VLAAction::REPORT_NOT_FOUND) {
        if (scan_progress < 1.0 || !all_visited(visited)) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "VLA reported not found before completing search coverage", target, started);
          return;
        }
        finish(
          goal_handle, false, FindObject::Result::TARGET_NOT_FOUND,
          "VLA completed coverage without confirming the target", target, started);
        return;
      }
    }
    finish(
      goal_handle, false, FindObject::Result::TIMEOUT,
      "VLA exceeded maximum high-level decision count", target, started);
  }

  void execute_continuous_policy(
    const std::shared_ptr<FindGoalHandle> & goal_handle, const OpenAiInstructionParser & parser,
    const TargetSpec & target, const std::vector<std::size_t> & candidates,
    const std::chrono::steady_clock::time_point & started)
  {
    // LLM may select only the next stage after an outcome; Nav2 remains the sole motion executor.
    std::vector<bool> visited(candidates.size(), false);
    std::optional<ContinuousVLAAction> previous_action;
    bool previous_goal_succeeded = true;
    std::string outcome = "initial";

    for (int stage_count = 0; stage_count < maximum_planner_stages_; ++stage_count) {
      if (goal_handle->is_canceling()) {
        finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
        return;
      }
      if (expired(started)) {
        finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
        return;
      }
      const auto allowed_stages = allowed_continuous_stages(outcome);
      if (allowed_stages.empty()) {
        finish(
          goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
          "continuous planner reached an unsupported stage transition", target, started);
        return;
      }

      StagePlan plan;
      try {
        plan = parser.plan_next_stage(
          goal_handle->get_goal()->instruction, target,
          StageContext{
            outcome, coverage_ratio(visited), detection_match_ratio(),
            target_distance_or_unknown()},
          allowed_stages);
      } catch (const std::exception & error) {
        finish(
          goal_handle, false, FindObject::Result::LANGUAGE_API_ERROR,
          std::string("stage planner request failed: ") + error.what(), target, started);
        return;
      }
      RCLCPP_INFO(
        get_logger(), "LLM stage=%s outcome=%s reason=%s", task_stage_name(plan.stage).c_str(),
        outcome.c_str(), plan.reason.c_str());

      if (plan.stage == TaskStage::SEARCH) {
        publish_feedback(
          goal_handle, FindObject::Feedback::SEARCHING_VIEWPOINTS, "searching_viewpoints",
          count_visited(visited), started, target.room, candidates.size());
        const auto stage_outcome = execute_continuous_search_stage(
          goal_handle, plan.stage_instruction, candidates, visited, previous_action,
          previous_goal_succeeded, started);
        if (stage_outcome == ContinuousStageOutcome::CANCELED) {
          finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
          return;
        }
        if (stage_outcome == ContinuousStageOutcome::TIMED_OUT) {
          finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
          return;
        }
        if (stage_outcome == ContinuousStageOutcome::FAILED) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "continuous VLA search local goal was rejected or Nav2 failed", target, started);
          return;
        }
        outcome = stage_outcome == ContinuousStageOutcome::TARGET_DETECTED ?
          "target_detected" : "coverage_complete";
        continue;
      }

      if (plan.stage == TaskStage::APPROACH) {
        publish_feedback(
          goal_handle, FindObject::Feedback::APPROACHING, "approaching",
          count_visited(visited), started, target.room, candidates.size());
        const auto stage_outcome = execute_continuous_approach_stage(
          goal_handle, plan.stage_instruction, candidates, visited, previous_action,
          previous_goal_succeeded, started);
        if (stage_outcome == ContinuousStageOutcome::CANCELED) {
          finish(goal_handle, false, FindObject::Result::CANCELED, "task canceled", target, started);
          return;
        }
        if (stage_outcome == ContinuousStageOutcome::TIMED_OUT) {
          finish(goal_handle, false, FindObject::Result::TIMEOUT, "task timeout", target, started);
          return;
        }
        if (stage_outcome == ContinuousStageOutcome::FAILED) {
          finish(
            goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
            "continuous VLA approach local goal was rejected or Nav2 failed", target, started);
          return;
        }
        outcome = stage_outcome == ContinuousStageOutcome::APPROACH_REACHED ?
          "approach_reached" : "detection_lost";
        continue;
      }

      if (plan.stage == TaskStage::VERIFY) {
        publish_feedback(
          goal_handle, FindObject::Feedback::VERIFYING, "verifying",
          count_visited(visited), started, target.room, candidates.size());
        outcome = verify_continuous_target(goal_handle, target, started) ?
          "target_verified" : "detection_lost";
        continue;
      }

      if (plan.stage == TaskStage::DONE && outcome == "target_verified") {
        geometry_msgs::msg::PoseStamped target_pose;
        float confidence = 0.0F;
        target_pose_in_map(target_pose, confidence);
        finish(
          goal_handle, true, FindObject::Result::NONE, "target found and verified", target,
          started, target_pose, confidence);
        return;
      }

      if (plan.stage == TaskStage::FAIL && outcome == "coverage_complete") {
        finish(
          goal_handle, false, FindObject::Result::TARGET_NOT_FOUND,
          "coverage completed without a stable target detection", target, started);
        return;
      }

      finish(
        goal_handle, false, FindObject::Result::NAVIGATION_FAILED,
        "LLM selected a stage that violates the continuous safety contract", target, started);
      return;
    }
    finish(
      goal_handle, false, FindObject::Result::TIMEOUT,
      "continuous planner exceeded the maximum stage count", target, started);
  }

  std::vector<TaskStage> allowed_continuous_stages(const std::string & outcome) const
  {
    if (outcome == "initial" || outcome == "detection_lost") {
      return {TaskStage::SEARCH};
    }
    if (outcome == "target_detected") {
      return {TaskStage::APPROACH};
    }
    if (outcome == "approach_reached") {
      return {TaskStage::VERIFY};
    }
    if (outcome == "target_verified") {
      return {TaskStage::DONE};
    }
    if (outcome == "coverage_complete") {
      return {TaskStage::FAIL};
    }
    return {};
  }

  ContinuousStageOutcome execute_continuous_search_stage(
    const std::shared_ptr<FindGoalHandle> & goal_handle, const std::string & stage_instruction,
    const std::vector<std::size_t> & candidates, std::vector<bool> & visited,
    std::optional<ContinuousVLAAction> & previous_action, bool & previous_goal_succeeded,
    const std::chrono::steady_clock::time_point & started)
  {
    for (int goal_count = 0; goal_count < maximum_local_goals_per_stage_; ++goal_count) {
      if (goal_handle->is_canceling()) {
        return ContinuousStageOutcome::CANCELED;
      }
      if (expired(started)) {
        return ContinuousStageOutcome::TIMED_OUT;
      }
      if (detection_confirmed()) {
        return ContinuousStageOutcome::TARGET_DETECTED;
      }
      if (all_visited(visited)) {
        return ContinuousStageOutcome::COVERAGE_COMPLETE;
      }
      try {
        const auto decision = request_continuous_policy_decision(
          stage_instruction,
          current_continuous_vla_state(
            candidates, visited, std::nullopt, 0.0F, previous_action, previous_goal_succeeded));
        if (!is_meaningful_local_action(decision.action)) {
          return ContinuousStageOutcome::FAILED;
        }
        RCLCPP_INFO(
          get_logger(), "VLA local goal dx=%.3f dy=%.3f yaw=%.3f model=%s latency=%.1fms",
          decision.action.delta_x_m, decision.action.delta_y_m, decision.action.delta_yaw_rad,
          decision.model_version.c_str(), decision.inference_milliseconds);
        previous_action = decision.action;
        const auto navigation = navigate_to(
          goal_handle, make_local_goal_pose(decision.action), started, true);
        previous_goal_succeeded = navigation == NavigationOutcome::SUCCEEDED;
        if (navigation == NavigationOutcome::TARGET_FOUND) {
          return ContinuousStageOutcome::TARGET_DETECTED;
        }
        if (navigation == NavigationOutcome::CANCELED) {
          return ContinuousStageOutcome::CANCELED;
        }
        if (navigation == NavigationOutcome::TIMED_OUT) {
          return ContinuousStageOutcome::TIMED_OUT;
        }
        if (navigation != NavigationOutcome::SUCCEEDED) {
          return ContinuousStageOutcome::FAILED;
        }
        mark_reached_candidates(candidates, visited);
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "continuous search decision failed: %s", error.what());
        return ContinuousStageOutcome::FAILED;
      }
    }
    return detection_confirmed() ?
           ContinuousStageOutcome::TARGET_DETECTED : ContinuousStageOutcome::FAILED;
  }

  ContinuousStageOutcome execute_continuous_approach_stage(
    const std::shared_ptr<FindGoalHandle> & goal_handle, const std::string & stage_instruction,
    const std::vector<std::size_t> & candidates, const std::vector<bool> & visited,
    std::optional<ContinuousVLAAction> & previous_action, bool & previous_goal_succeeded,
    const std::chrono::steady_clock::time_point & started)
  {
    for (int goal_count = 0; goal_count < maximum_local_goals_per_stage_; ++goal_count) {
      if (goal_handle->is_canceling()) {
        return ContinuousStageOutcome::CANCELED;
      }
      if (expired(started)) {
        return ContinuousStageOutcome::TIMED_OUT;
      }
      geometry_msgs::msg::PoseStamped target_pose;
      float confidence = 0.0F;
      if (!detection_confirmed() || !target_pose_in_map(target_pose, confidence)) {
        return ContinuousStageOutcome::DETECTION_LOST;
      }
      if (distance_to(target_pose) <= approach_distance_) {
        return ContinuousStageOutcome::APPROACH_REACHED;
      }
      try {
        const auto decision = request_continuous_policy_decision(
          stage_instruction,
          current_continuous_vla_state(
            candidates, visited, target_pose, confidence, previous_action, previous_goal_succeeded));
        if (!is_meaningful_local_action(decision.action)) {
          return ContinuousStageOutcome::FAILED;
        }
        previous_action = decision.action;
        const auto navigation = navigate_to(
          goal_handle, make_local_goal_pose(decision.action), started, false);
        previous_goal_succeeded = navigation == NavigationOutcome::SUCCEEDED;
        if (navigation == NavigationOutcome::CANCELED) {
          return ContinuousStageOutcome::CANCELED;
        }
        if (navigation == NavigationOutcome::TIMED_OUT) {
          return ContinuousStageOutcome::TIMED_OUT;
        }
        if (navigation != NavigationOutcome::SUCCEEDED) {
          return ContinuousStageOutcome::FAILED;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "continuous approach decision failed: %s", error.what());
        return ContinuousStageOutcome::FAILED;
      }
    }
    return ContinuousStageOutcome::FAILED;
  }

  bool verify_continuous_target(
    const std::shared_ptr<FindGoalHandle> & goal_handle, const TargetSpec & target,
    const std::chrono::steady_clock::time_point & started)
  {
    reset_detection(target.object_class + ":" + target.color);
    const auto verify_started = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - verify_started).count() <
      verification_timeout_)
    {
      if (goal_handle->is_canceling() || expired(started)) {
        return false;
      }
      if (detection_confirmed()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  VLAState current_vla_state(
    const std::vector<std::size_t> & candidates, const std::vector<bool> & visited,
    const int last_action_id, const double scan_progress) const
  {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    std::vector<VLAViewpoint> policy_viewpoints;
    policy_viewpoints.reserve(candidates.size());
    for (const auto index : candidates) {
      policy_viewpoints.push_back(VLAViewpoint{
        index, viewpoints_[index * 3], viewpoints_[index * 3 + 1], viewpoints_[index * 3 + 2]});
    }
    return build_vla_policy_state(
      transform.transform.translation.x, transform.transform.translation.y,
      tf2::getYaw(transform.transform.rotation), policy_viewpoints, visited,
      last_action_id, scan_progress);
  }

  ContinuousVLAState current_continuous_vla_state(
    const std::vector<std::size_t> & candidates, const std::vector<bool> & visited,
    const std::optional<geometry_msgs::msg::PoseStamped> & target_pose,
    const float target_confidence, const std::optional<ContinuousVLAAction> & previous_action,
    const bool previous_goal_succeeded) const
  {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    std::vector<VLAViewpoint> policy_viewpoints;
    policy_viewpoints.reserve(candidates.size());
    for (const auto index : candidates) {
      policy_viewpoints.push_back(VLAViewpoint{
        index, viewpoints_[index * 3], viewpoints_[index * 3 + 1], viewpoints_[index * 3 + 2]});
    }
    std::optional<VLAViewpoint> policy_target;
    if (target_pose.has_value()) {
      policy_target = VLAViewpoint{
        0U, target_pose->pose.position.x, target_pose->pose.position.y, 0.0};
    }
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_) {
        throw std::runtime_error("continuous VLA requires an odometry sample");
      }
      linear_velocity = linear_velocity_;
      angular_velocity = angular_velocity_;
    }
    return build_continuous_vla_policy_state(
      transform.transform.translation.x, transform.transform.translation.y,
      tf2::getYaw(transform.transform.rotation), policy_viewpoints, visited, policy_target,
      target_confidence, linear_velocity, angular_velocity,
      previous_action.value_or(ContinuousVLAAction{0.0, 0.0, 0.0}),
      detection_match_ratio(), previous_goal_succeeded);
  }

  VLAActionDecision request_policy_decision(
    const std::string & instruction, const VLAState & state) const
  {
    if (!policy_client_.has_value()) {
      throw std::logic_error("VLA policy client is unavailable");
    }
    return policy_client_->select(
      instruction, latest_policy_jpeg(), std::vector<double>(state.values.begin(), state.values.end()));
  }

  ContinuousVLAActionDecision request_continuous_policy_decision(
    const std::string & stage_instruction, const ContinuousVLAState & state) const
  {
    if (!continuous_policy_client_.has_value()) {
      throw std::logic_error("continuous VLA policy client is unavailable");
    }
    return continuous_policy_client_->select(
      stage_instruction, latest_policy_jpeg(),
      std::vector<double>(state.values.begin(), state.values.end()));
  }

  std::vector<uint8_t> latest_policy_jpeg() const
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!latest_image_ || !latest_image_received_at_.has_value() ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
      *latest_image_received_at_).count() > policy_image_max_age_)
    {
      throw std::runtime_error("latest policy image is missing or stale");
    }
    const auto image = cv_bridge::toCvCopy(latest_image_, "bgr8");
    std::vector<uint8_t> jpeg;
    if (!cv::imencode(".jpg", image->image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 90})) {
      throw std::runtime_error("failed to JPEG-encode policy image");
    }
    return jpeg;
  }

  void log_shadow_decision(
    const std::string & instruction, const std::vector<std::size_t> & candidates,
    const std::vector<bool> & visited, const int last_action_id, const double scan_progress) const
  {
    try {
      const auto state = current_vla_state(candidates, visited, last_action_id, scan_progress);
      const auto decision = request_policy_decision(instruction, state);
      RCLCPP_INFO(
        get_logger(), "VLA shadow action=%s distance=%.3f gap=%.3f model=%s",
        vla_action_name(decision.action).c_str(), decision.prototype_distance,
        decision.prototype_gap, decision.model_version.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "VLA shadow decision was unavailable: %s", error.what());
    }
  }

  void log_continuous_shadow_decision(
    const std::string & stage_instruction, const std::vector<std::size_t> & candidates,
    const std::vector<bool> & visited,
    const std::optional<geometry_msgs::msg::PoseStamped> & target_pose,
    const std::optional<ContinuousVLAAction> & previous_action,
    const bool previous_goal_succeeded) const
  {
    try {
      const auto decision = request_continuous_policy_decision(
        stage_instruction,
        current_continuous_vla_state(
          candidates, visited, target_pose, 0.0F, previous_action, previous_goal_succeeded));
      RCLCPP_INFO(
        get_logger(), "VLA shadow local goal dx=%.3f dy=%.3f yaw=%.3f model=%s latency=%.1fms",
        decision.action.delta_x_m, decision.action.delta_y_m, decision.action.delta_yaw_rad,
        decision.model_version.c_str(), decision.inference_milliseconds);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "continuous VLA shadow decision was unavailable: %s", error.what());
    }
  }

  bool policy_decision_accepted(const VLAActionDecision & decision) const
  {
    return decision.prototype_distance <= policy_max_prototype_distance_ &&
           decision.prototype_gap >= policy_min_prototype_gap_;
  }

  static std::optional<std::size_t> candidate_position(
    const std::vector<std::size_t> & candidates, const std::size_t index)
  {
    const auto iterator = std::find(candidates.begin(), candidates.end(), index);
    if (iterator == candidates.end()) {
      return std::nullopt;
    }
    return static_cast<std::size_t>(iterator - candidates.begin());
  }

  static std::size_t count_visited(const std::vector<bool> & visited)
  {
    return static_cast<std::size_t>(std::count(visited.begin(), visited.end(), true));
  }

  static bool all_visited(const std::vector<bool> & visited)
  {
    return std::all_of(visited.begin(), visited.end(), [](const bool value) {return value;});
  }

  static double coverage_ratio(const std::vector<bool> & visited)
  {
    if (visited.empty()) {
      return 1.0;
    }
    return static_cast<double>(count_visited(visited)) / static_cast<double>(visited.size());
  }

  double detection_match_ratio() const
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    return static_cast<double>(detection_history_.match_count()) / 15.0;
  }

  double target_distance_or_unknown()
  {
    geometry_msgs::msg::PoseStamped target_pose;
    float confidence = 0.0F;
    if (!detection_confirmed() || !target_pose_in_map(target_pose, confidence)) {
      return -1.0;
    }
    try {
      return distance_to(target_pose);
    } catch (const tf2::TransformException &) {
      return -1.0;
    }
  }

  double distance_to(const geometry_msgs::msg::PoseStamped & target) const
  {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    return std::hypot(
      target.pose.position.x - transform.transform.translation.x,
      target.pose.position.y - transform.transform.translation.y);
  }

  static bool is_meaningful_local_action(const ContinuousVLAAction & action)
  {
    return std::hypot(action.delta_x_m, action.delta_y_m) >= 0.05 ||
           std::abs(action.delta_yaw_rad) >= 0.05;
  }

  geometry_msgs::msg::PoseStamped make_local_goal_pose(const ContinuousVLAAction & action) const
  {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    const double yaw = tf2::getYaw(transform.transform.rotation);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = transform.transform.translation.x +
      std::cos(yaw) * action.delta_x_m - std::sin(yaw) * action.delta_y_m;
    pose.pose.position.y = transform.transform.translation.y +
      std::sin(yaw) * action.delta_x_m + std::cos(yaw) * action.delta_y_m;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, normalize_angle(yaw + action.delta_yaw_rad));
    pose.pose.orientation = tf2::toMsg(orientation);
    return pose;
  }

  void mark_reached_candidates(
    const std::vector<std::size_t> & candidates, std::vector<bool> & visited) const
  {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    constexpr double kViewpointReachedToleranceM = 0.45;
    for (std::size_t position = 0; position < candidates.size(); ++position) {
      const std::size_t index = candidates[position];
      if (!visited[position] && std::hypot(
          viewpoints_[index * 3] - transform.transform.translation.x,
          viewpoints_[index * 3 + 1] - transform.transform.translation.y) <=
        kViewpointReachedToleranceM)
      {
        visited[position] = true;
      }
    }
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

  NavigationOutcome spin_for_scan(
    const std::shared_ptr<FindGoalHandle> & task, const std::size_t viewpoint_index,
    const bool is_left, const std::chrono::steady_clock::time_point & task_started)
  {
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(5))) {
      return NavigationOutcome::FAILED;
    }
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    constexpr double kScanOffset = 0.78539816339744830962;
    const double target_yaw = viewpoints_[viewpoint_index * 3 + 2] + (is_left ? kScanOffset : -kScanOffset);
    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(normalize_angle(target_yaw - tf2::getYaw(transform.transform.rotation)));
    goal.time_allowance = duration_from_seconds(std::min(navigation_timeout_, 30.0));
    const auto goal_future = spin_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return NavigationOutcome::FAILED;
    }
    const auto spin_goal = goal_future.get();
    if (!spin_goal) {
      return NavigationOutcome::FAILED;
    }
    const auto result_future = spin_client_->async_get_result(spin_goal);
    const auto spin_started = std::chrono::steady_clock::now();
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (task->is_canceling()) {
        cancel_spin(spin_goal);
        return NavigationOutcome::CANCELED;
      }
      if (expired(task_started)) {
        cancel_spin(spin_goal);
        return NavigationOutcome::TIMED_OUT;
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - spin_started).count() >
        navigation_timeout_)
      {
        cancel_spin(spin_goal);
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

  void cancel_spin(const SpinGoalHandle::SharedPtr & goal)
  {
    const auto cancel_future = spin_client_->async_cancel_goal(goal);
    if (cancel_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Nav2 did not acknowledge spin cancellation within 2 seconds");
    }
  }

  static double normalize_angle(double angle)
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

  static builtin_interfaces::msg::Duration duration_from_seconds(const double seconds)
  {
    builtin_interfaces::msg::Duration result;
    result.sec = static_cast<int32_t>(seconds);
    result.nanosec = static_cast<uint32_t>((seconds - result.sec) * 1000000000.0);
    return result;
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
      latest_detection_received_at_ = std::chrono::steady_clock::now();
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = message;
    latest_image_received_at_ = std::chrono::steady_clock::now();
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    linear_velocity_ = message->twist.twist.linear.x;
    angular_velocity_ = message->twist.twist.angular.z;
    has_odom_ = true;
  }

  void reset_detection(const std::string & target_label)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    target_label_ = target_label;
    detection_history_.reset();
    latest_detection_ = geometry_msgs::msg::PoseStamped();
    latest_detection_received_at_.reset();
    best_confidence_ = 0.0F;
  }

  bool detection_confirmed()
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    return detection_history_.confirmed() && latest_detection_received_at_.has_value() &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
           *latest_detection_received_at_).count() <= detection_max_age_;
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
  int maximum_policy_decisions_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string policy_mode_;
  std::string policy_endpoint_;
  std::string policy_interface_;
  std::string continuous_policy_endpoint_;
  int policy_timeout_milliseconds_;
  double policy_max_prototype_distance_;
  double policy_min_prototype_gap_;
  double policy_image_max_age_;
  double detection_max_age_;
  int maximum_planner_stages_;
  int maximum_local_goals_per_stage_;
  std::string camera_topic_;
  std::atomic_bool is_executing_{false};

  mutable std::mutex detection_mutex_;
  DetectionHistory detection_history_;
  std::string target_label_;
  geometry_msgs::msg::PoseStamped latest_detection_;
  std::optional<std::chrono::steady_clock::time_point> latest_detection_received_at_;
  float best_confidence_{0.0F};

  mutable std::mutex image_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::optional<std::chrono::steady_clock::time_point> latest_image_received_at_;

  mutable std::mutex odom_mutex_;
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  bool has_odom_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<VLAActionPolicyClient> policy_client_;
  std::optional<ContinuousVLAActionPolicyClient> continuous_policy_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Server<FindObject>::SharedPtr action_server_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
