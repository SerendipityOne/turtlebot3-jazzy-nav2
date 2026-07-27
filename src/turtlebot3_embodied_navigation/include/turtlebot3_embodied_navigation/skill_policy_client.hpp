// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#ifndef TURTLEBOT3_EMBODIED_NAVIGATION__SKILL_POLICY_CLIENT_HPP_
#define TURTLEBOT3_EMBODIED_NAVIGATION__SKILL_POLICY_CLIENT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace turtlebot3_embodied_navigation
{

struct SkillDecision
{
  std::string skill;
  float confidence;
};

SkillDecision parse_skill_response(const std::string & response);

class SkillPolicyClient
{
public:
  SkillPolicyClient(std::string endpoint, long timeout_milliseconds);

  SkillDecision select(
    const std::string & instruction, const std::vector<uint8_t> & jpeg,
    const std::vector<double> & robot_state) const;

private:
  std::string endpoint_;
  long timeout_milliseconds_;
};

}  // namespace turtlebot3_embodied_navigation

#endif  // TURTLEBOT3_EMBODIED_NAVIGATION__SKILL_POLICY_CLIENT_HPP_
