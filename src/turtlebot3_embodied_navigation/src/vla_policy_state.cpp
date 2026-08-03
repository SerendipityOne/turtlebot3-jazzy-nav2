// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/vla_policy_state.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace turtlebot3_embodied_navigation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

}  // namespace

VLAState build_vla_policy_state(
  const double robot_x, const double robot_y, const double robot_yaw,
  const std::vector<VLAViewpoint> & viewpoints, const std::vector<bool> & visited,
  const int last_action_id, const double scan_progress)
{
  if (!std::isfinite(robot_x) || !std::isfinite(robot_y) || !std::isfinite(robot_yaw) ||
    !std::isfinite(scan_progress) || scan_progress < 0.0 || scan_progress > 1.0 ||
    viewpoints.size() != visited.size() || last_action_id < -1 ||
    last_action_id >= static_cast<int>(kVlaActionCount))
  {
    throw std::invalid_argument("VLA policy state input is invalid");
  }

  struct Candidate
  {
    std::size_t index;
    double distance;
    double bearing;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(viewpoints.size());
  for (std::size_t position = 0; position < viewpoints.size(); ++position) {
    if (visited[position]) {
      continue;
    }
    const auto & viewpoint = viewpoints[position];
    if (!std::isfinite(viewpoint.x) || !std::isfinite(viewpoint.y) || !std::isfinite(viewpoint.yaw)) {
      throw std::invalid_argument("VLA viewpoint contains a non-finite value");
    }
    const double dx = viewpoint.x - robot_x;
    const double dy = viewpoint.y - robot_y;
    candidates.push_back(Candidate{
      position, std::hypot(dx, dy), normalize_angle(std::atan2(dy, dx) - robot_yaw)});
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & left, const Candidate & right) {
      return left.distance == right.distance ? left.index < right.index : left.distance < right.distance;
    });

  VLAState state;
  for (std::size_t slot = 0; slot < kVlaCandidateSlots && slot < candidates.size(); ++slot) {
    const auto & candidate = candidates[slot];
    const std::size_t offset = slot * 4;
    state.values[offset] = candidate.distance;
    state.values[offset + 1] = std::sin(candidate.bearing);
    state.values[offset + 2] = std::cos(candidate.bearing);
    state.values[offset + 3] = 1.0;
    state.candidate_indices[slot] = viewpoints[candidate.index].index;
  }
  if (last_action_id >= 0) {
    state.values[12 + static_cast<std::size_t>(last_action_id)] = 1.0;
  }
  state.values.back() = scan_progress;
  return state;
}

ContinuousVLAState build_continuous_vla_policy_state(
  const double robot_x, const double robot_y, const double robot_yaw,
  const std::vector<VLAViewpoint> & viewpoints, const std::vector<bool> & visited,
  const std::optional<VLAViewpoint> & target, const double target_confidence,
  const double linear_velocity, const double angular_velocity,
  const ContinuousVLAAction & previous_action, const double detection_match_ratio,
  const bool last_goal_succeeded)
{
  if (!std::isfinite(target_confidence) || target_confidence < 0.0 ||
    !std::isfinite(linear_velocity) || !std::isfinite(angular_velocity) ||
    !std::isfinite(previous_action.delta_x_m) || !std::isfinite(previous_action.delta_y_m) ||
    !std::isfinite(previous_action.delta_yaw_rad) || !std::isfinite(detection_match_ratio) ||
    detection_match_ratio < 0.0 || detection_match_ratio > 1.0)
  {
    throw std::invalid_argument("continuous VLA policy state input is invalid");
  }

  const VLAState candidate_state = build_vla_policy_state(
    robot_x, robot_y, robot_yaw, viewpoints, visited, -1, 1.0);
  ContinuousVLAState state;
  std::copy_n(candidate_state.values.begin(), 12, state.values.begin());

  constexpr std::size_t kTargetOffset = 12;
  if (target.has_value()) {
    if (!std::isfinite(target->x) || !std::isfinite(target->y)) {
      throw std::invalid_argument("continuous VLA target contains a non-finite value");
    }
    const double map_dx = target->x - robot_x;
    const double map_dy = target->y - robot_y;
    state.values[kTargetOffset] = std::cos(robot_yaw) * map_dx + std::sin(robot_yaw) * map_dy;
    state.values[kTargetOffset + 1] = -std::sin(robot_yaw) * map_dx + std::cos(robot_yaw) * map_dy;
    state.values[kTargetOffset + 2] = std::hypot(map_dx, map_dy);
    state.values[kTargetOffset + 3] = target_confidence;
    state.values[kTargetOffset + 4] = 1.0;
  }

  state.values[17] = linear_velocity;
  state.values[18] = angular_velocity;
  state.values[19] = previous_action.delta_x_m;
  state.values[20] = previous_action.delta_y_m;
  state.values[21] = previous_action.delta_yaw_rad;
  state.values[22] = viewpoints.empty() ? 1.0 :
    static_cast<double>(std::count(visited.begin(), visited.end(), true)) / viewpoints.size();
  state.values[23] = detection_match_ratio;
  state.values[24] = last_goal_succeeded ? 1.0 : 0.0;
  return state;
}

}  // namespace turtlebot3_embodied_navigation
