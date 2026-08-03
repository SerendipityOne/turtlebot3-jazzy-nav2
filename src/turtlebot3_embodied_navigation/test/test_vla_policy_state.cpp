// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "turtlebot3_embodied_navigation/vla_policy_state.hpp"

TEST(VLAState, EncodesNearestUnvisitedCandidatesInRobotFrame)
{
  using turtlebot3_embodied_navigation::VLAViewpoint;
  const std::vector<VLAViewpoint> viewpoints{
    {10, 1.0, 0.0, 0.0}, {11, 0.0, 2.0, 0.0}, {12, -1.0, 0.0, 0.0}};
  const auto state = turtlebot3_embodied_navigation::build_vla_policy_state(
    0.0, 0.0, 0.0, viewpoints, {true, false, false}, 4, 0.5);

  ASSERT_TRUE(state.candidate_indices[0].has_value());
  ASSERT_TRUE(state.candidate_indices[1].has_value());
  EXPECT_EQ(*state.candidate_indices[0], 12U);
  EXPECT_EQ(*state.candidate_indices[1], 11U);
  EXPECT_DOUBLE_EQ(state.values[0], 1.0);
  EXPECT_NEAR(state.values[1], 0.0, 1e-9);
  EXPECT_NEAR(state.values[2], -1.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.values[3], 1.0);
  EXPECT_DOUBLE_EQ(state.values[16], 1.0);
  EXPECT_DOUBLE_EQ(state.values.back(), 0.5);
}

TEST(VLAState, RejectsInvalidScanProgress)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::build_vla_policy_state(
      0.0, 0.0, 0.0, {}, {}, -1, 1.1),
    std::invalid_argument);
}

TEST(ContinuousVLAState, EncodesTargetAndContinuousContextInRobotFrame)
{
  using turtlebot3_embodied_navigation::ContinuousVLAAction;
  using turtlebot3_embodied_navigation::VLAViewpoint;
  const std::vector<VLAViewpoint> viewpoints{{10, 1.0, 0.0, 0.0}};
  const auto state = turtlebot3_embodied_navigation::build_continuous_vla_policy_state(
    0.0, 0.0, 0.0, viewpoints, {false}, VLAViewpoint{99, 2.0, 1.0, 0.0}, 0.9,
    0.2, -0.1, ContinuousVLAAction{0.3, 0.1, -0.2}, 7.0 / 15.0, true);

  EXPECT_DOUBLE_EQ(state.values[0], 1.0);
  EXPECT_DOUBLE_EQ(state.values[12], 2.0);
  EXPECT_DOUBLE_EQ(state.values[13], 1.0);
  EXPECT_NEAR(state.values[14], std::sqrt(5.0), 1e-9);
  EXPECT_DOUBLE_EQ(state.values[15], 0.9);
  EXPECT_DOUBLE_EQ(state.values[16], 1.0);
  EXPECT_DOUBLE_EQ(state.values[17], 0.2);
  EXPECT_DOUBLE_EQ(state.values[18], -0.1);
  EXPECT_DOUBLE_EQ(state.values[19], 0.3);
  EXPECT_DOUBLE_EQ(state.values[22], 0.0);
  EXPECT_DOUBLE_EQ(state.values[23], 7.0 / 15.0);
  EXPECT_DOUBLE_EQ(state.values[24], 1.0);
}
