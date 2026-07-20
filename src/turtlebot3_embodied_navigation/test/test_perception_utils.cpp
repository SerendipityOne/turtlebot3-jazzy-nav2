// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <cmath>

#include "turtlebot3_embodied_navigation/perception_utils.hpp"

namespace tb3 = turtlebot3_embodied_navigation;

TEST(PerceptionUtils, UsesMedianValidDepthInCenterRegion)
{
  cv::Mat depth(20, 20, CV_32FC1, cv::Scalar(2.0F));
  depth.at<float>(10, 10) = 100.0F;
  depth.at<float>(11, 11) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FLOAT_EQ(tb3::median_depth(depth, cv::Rect(4, 4, 12, 12), 0.1F, 8.0F), 2.0F);
}

TEST(PerceptionUtils, ProjectsOpticalPixelToThreeDimensions)
{
  const auto point = tb3::project_pixel(420.0, 290.0, 2.0, 500.0, 500.0, 320.0, 240.0);
  EXPECT_NEAR(point.x, 0.4, 1e-9);
  EXPECT_NEAR(point.y, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(point.z, 2.0);
}

TEST(PerceptionUtils, ClassifiesConfiguredColors)
{
  cv::Mat red(20, 20, CV_8UC3, cv::Scalar(0, 0, 255));
  cv::Mat green(20, 20, CV_8UC3, cv::Scalar(0, 255, 0));
  cv::Mat blue(20, 20, CV_8UC3, cv::Scalar(255, 0, 0));
  EXPECT_EQ(tb3::classify_color(red, cv::Rect(0, 0, 20, 20)), "red");
  EXPECT_EQ(tb3::classify_color(green, cv::Rect(0, 0, 20, 20)), "green");
  EXPECT_EQ(tb3::classify_color(blue, cv::Rect(0, 0, 20, 20)), "blue");
}
