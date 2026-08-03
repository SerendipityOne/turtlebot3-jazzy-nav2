// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#ifndef TURTLEBOT3_EMBODIED_NAVIGATION__VLA_POLICY_STATE_HPP_
#define TURTLEBOT3_EMBODIED_NAVIGATION__VLA_POLICY_STATE_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace turtlebot3_embodied_navigation
{

constexpr std::size_t kVlaCandidateSlots = 3;
constexpr std::size_t kVlaActionCount = 10;
constexpr std::size_t kVlaPolicyStateSize = 23;
constexpr std::size_t kContinuousVlaActionSize = 3;
constexpr std::size_t kContinuousVlaPolicyStateSize = 25;

struct VLAViewpoint
{
  std::size_t index;
  double x;
  double y;
  double yaw;
};

// State is map-independent: candidate slots are expressed in the robot frame.
struct VLAState
{
  std::array<double, kVlaPolicyStateSize> values{};
  std::array<std::optional<std::size_t>, kVlaCandidateSlots> candidate_indices{};
};

// Continuous policy observations stay map-independent: map poses are converted
// to robot-frame candidates and target coordinates before reaching the model.
struct ContinuousVLAState
{
  std::array<double, kContinuousVlaPolicyStateSize> values{};
};

struct ContinuousVLAAction
{
  double delta_x_m;
  double delta_y_m;
  double delta_yaw_rad;
};

VLAState build_vla_policy_state(
  double robot_x, double robot_y, double robot_yaw,
  const std::vector<VLAViewpoint> & viewpoints, const std::vector<bool> & visited,
  int last_action_id, double scan_progress);

ContinuousVLAState build_continuous_vla_policy_state(
  double robot_x, double robot_y, double robot_yaw,
  const std::vector<VLAViewpoint> & viewpoints, const std::vector<bool> & visited,
  const std::optional<VLAViewpoint> & target, double target_confidence,
  double linear_velocity, double angular_velocity, const ContinuousVLAAction & previous_action,
  double detection_match_ratio, bool last_goal_succeeded);

}  // namespace turtlebot3_embodied_navigation

#endif  // TURTLEBOT3_EMBODIED_NAVIGATION__VLA_POLICY_STATE_HPP_
