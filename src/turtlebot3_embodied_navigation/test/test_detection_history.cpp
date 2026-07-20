// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include "turtlebot3_embodied_navigation/detection_history.hpp"

using turtlebot3_embodied_navigation::DetectionHistory;

TEST(DetectionHistory, RequiresSevenMatchesInLatestFifteenFrames)
{
  DetectionHistory history(15, 7);

  for (int index = 0; index < 6; ++index) {
    EXPECT_FALSE(history.add(true));
  }
  EXPECT_TRUE(history.add(true));
}

TEST(DetectionHistory, DropsMatchesOutsideWindow)
{
  DetectionHistory history(15, 7);
  for (int index = 0; index < 7; ++index) {
    history.add(true);
  }
  for (int index = 0; index < 8; ++index) {
    history.add(false);
  }
  EXPECT_TRUE(history.confirmed());

  EXPECT_FALSE(history.add(false));
  EXPECT_EQ(history.match_count(), 6U);
  EXPECT_EQ(history.size(), 15U);
}

TEST(DetectionHistory, RejectsInvalidConfiguration)
{
  EXPECT_THROW(DetectionHistory(0, 0), std::invalid_argument);
  EXPECT_THROW(DetectionHistory(15, 16), std::invalid_argument);
}
