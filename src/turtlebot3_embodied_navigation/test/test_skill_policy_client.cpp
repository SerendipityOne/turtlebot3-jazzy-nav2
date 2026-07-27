// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "turtlebot3_embodied_navigation/skill_policy_client.hpp"

TEST(SkillPolicyClient, ParsesAllowedSkill)
{
  const auto decision = turtlebot3_embodied_navigation::parse_skill_response(
    R"({"skill":"rotate_scan","confidence":0.75})");
  EXPECT_EQ(decision.skill, "rotate_scan");
  EXPECT_FLOAT_EQ(decision.confidence, 0.75F);
}

TEST(SkillPolicyClient, RejectsUnknownSkill)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_skill_response(
      R"({"skill":"publish_cmd_vel","confidence":0.9})"),
    std::invalid_argument);
}

TEST(SkillPolicyClient, RejectsInvalidConfidence)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_skill_response(
      R"({"skill":"stop","confidence":1.1})"),
    std::invalid_argument);
}

TEST(SkillPolicyClient, RequiresLoopbackEndpoint)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::SkillPolicyClient(
      "https://remote.example/v1/select_skill", 500),
    std::invalid_argument);
}
