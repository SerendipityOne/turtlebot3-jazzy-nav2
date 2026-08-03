// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#ifndef TURTLEBOT3_EMBODIED_NAVIGATION__SKILL_POLICY_CLIENT_HPP_
#define TURTLEBOT3_EMBODIED_NAVIGATION__SKILL_POLICY_CLIENT_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "turtlebot3_embodied_navigation/vla_policy_state.hpp"

namespace turtlebot3_embodied_navigation
{

enum class VLAAction : std::uint8_t
{
  GO_TO_CANDIDATE_0 = 0,
  GO_TO_CANDIDATE_1 = 1,
  GO_TO_CANDIDATE_2 = 2,
  SCAN_LEFT = 3,
  SCAN_RIGHT = 4,
  APPROACH_RED = 5,
  APPROACH_GREEN = 6,
  APPROACH_BLUE = 7,
  APPROACH_YELLOW = 8,
  REPORT_NOT_FOUND = 9,
};

struct VLAActionDecision
{
  VLAAction action;
  float prototype_distance;
  float prototype_gap;
  std::string model_version;
};

struct ContinuousVLAActionDecision
{
  ContinuousVLAAction action;
  std::string model_version;
  float inference_milliseconds;
};

VLAActionDecision parse_vla_policy_response(const std::string & response);
ContinuousVLAActionDecision parse_continuous_vla_policy_response(const std::string & response);
std::string vla_action_name(VLAAction action);
std::optional<std::string> vla_approach_color(VLAAction action);
bool is_vla_candidate_action(VLAAction action);
std::size_t vla_candidate_slot(VLAAction action);

class VLAActionPolicyClient
{
public:
  VLAActionPolicyClient(std::string endpoint, long timeout_milliseconds);

  VLAActionDecision select(
    const std::string & instruction, const std::vector<uint8_t> & jpeg,
    const std::vector<double> & robot_state) const;

private:
  std::string endpoint_;
  long timeout_milliseconds_;
};

class ContinuousVLAActionPolicyClient
{
public:
  ContinuousVLAActionPolicyClient(std::string endpoint, long timeout_milliseconds);

  ContinuousVLAActionDecision select(
    const std::string & stage_instruction, const std::vector<uint8_t> & jpeg,
    const std::vector<double> & robot_state) const;

private:
  std::string endpoint_;
  long timeout_milliseconds_;
};

}  // namespace turtlebot3_embodied_navigation

#endif  // TURTLEBOT3_EMBODIED_NAVIGATION__SKILL_POLICY_CLIENT_HPP_
