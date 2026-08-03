// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "turtlebot3_embodied_navigation/skill_policy_client.hpp"

TEST(VLAActionPolicyClient, ParsesSupportedAction)
{
  const auto decision = turtlebot3_embodied_navigation::parse_vla_policy_response(
    R"({"schema_version":1,"action_id":7,"prototype_distance":0.12,"prototype_gap":0.75,"model_version":"test"})");
  EXPECT_EQ(decision.action, turtlebot3_embodied_navigation::VLAAction::APPROACH_BLUE);
  EXPECT_FLOAT_EQ(decision.prototype_distance, 0.12F);
  EXPECT_FLOAT_EQ(decision.prototype_gap, 0.75F);
  EXPECT_EQ(decision.model_version, "test");
}

TEST(VLAActionPolicyClient, RejectsUnknownAction)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_vla_policy_response(
      R"({"schema_version":1,"action_id":10,"prototype_distance":0.1,"prototype_gap":0.9,"model_version":"test"})"),
    std::invalid_argument);
}

TEST(VLAActionPolicyClient, RejectsInvalidPrototypeMetrics)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_vla_policy_response(
      R"({"schema_version":1,"action_id":9,"prototype_distance":-0.1,"prototype_gap":0.5,"model_version":"test"})"),
    std::invalid_argument);
}

TEST(VLAActionPolicyClient, RejectsEmptyModelVersion)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_vla_policy_response(
      R"({"schema_version":1,"action_id":9,"prototype_distance":0.1,"prototype_gap":0.5,"model_version":""})"),
    std::invalid_argument);
}

TEST(VLAActionPolicyClient, RequiresLoopbackEndpoint)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::VLAActionPolicyClient(
      "https://remote.example/v1/select_skill", 500),
    std::invalid_argument);
}

TEST(ContinuousVLAActionPolicyClient, ParsesBoundedLocalGoal)
{
  const auto decision = turtlebot3_embodied_navigation::parse_continuous_vla_policy_response(
    R"({"schema_version":2,"delta_x_m":0.5,"delta_y_m":-0.25,"delta_yaw_rad":0.4,"model_version":"continuous-test","inference_milliseconds":18.5})");
  EXPECT_DOUBLE_EQ(decision.action.delta_x_m, 0.5);
  EXPECT_DOUBLE_EQ(decision.action.delta_y_m, -0.25);
  EXPECT_DOUBLE_EQ(decision.action.delta_yaw_rad, 0.4);
  EXPECT_EQ(decision.model_version, "continuous-test");
}

TEST(ContinuousVLAActionPolicyClient, RejectsUnsafeLocalGoal)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_continuous_vla_policy_response(
      R"({"schema_version":2,"delta_x_m":0.8,"delta_y_m":0.0,"delta_yaw_rad":0.0,"model_version":"test","inference_milliseconds":1.0})"),
    std::invalid_argument);
}
