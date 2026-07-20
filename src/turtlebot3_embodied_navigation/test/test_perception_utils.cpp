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

TEST(PerceptionUtils, DecodesYolo11ChannelsFirstOutputAndLetterboxPadding)
{
  const int shape[] = {1, 6, 2};
  cv::Mat output(3, shape, CV_32F, cv::Scalar(0.0F));
  float * values = output.ptr<float>();
  const auto set = [values](const int channel, const int candidate, const float value) {
      values[channel * 2 + candidate] = value;
    };
  set(0, 0, 320.0F);
  set(1, 0, 180.0F);
  set(2, 0, 100.0F);
  set(3, 0, 40.0F);
  set(4, 0, 0.90F);
  set(5, 0, 0.10F);

  const auto detections = tb3::decode_yolo11(
    output, 1.0, cv::Size(640, 480), 0, 80, 2, 0.5F, 0.45F);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().class_index, 0);
  EXPECT_FLOAT_EQ(detections.front().confidence, 0.90F);
  EXPECT_EQ(detections.front().box, cv::Rect(270, 80, 100, 40));
}

TEST(PerceptionUtils, KeepsOverlappingBoxesFromDifferentClasses)
{
  const int shape[] = {1, 6, 2};
  cv::Mat output(3, shape, CV_32F, cv::Scalar(0.0F));
  float * values = output.ptr<float>();
  for (int candidate = 0; candidate < 2; ++candidate) {
    values[0 * 2 + candidate] = 100.0F;
    values[1 * 2 + candidate] = 100.0F;
    values[2 * 2 + candidate] = 40.0F;
    values[3 * 2 + candidate] = 40.0F;
  }
  values[4 * 2] = 0.9F;
  values[5 * 2 + 1] = 0.8F;

  const auto detections = tb3::decode_yolo11(
    output, 1.0, cv::Size(640, 480), 0, 0, 2, 0.5F, 0.45F);
  EXPECT_EQ(detections.size(), 2U);
}

TEST(PerceptionUtils, DecodesYolo11ChannelsLastOutput)
{
  const int shape[] = {1, 2, 6};
  cv::Mat output(3, shape, CV_32F, cv::Scalar(0.0F));
  float * values = output.ptr<float>();
  values[0] = 50.0F;
  values[1] = 60.0F;
  values[2] = 20.0F;
  values[3] = 30.0F;
  values[4] = 0.10F;
  values[5] = 0.85F;

  const auto detections = tb3::decode_yolo11(
    output, 1.0, cv::Size(640, 480), 0, 0, 2, 0.5F, 0.45F);
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().class_index, 1);
  EXPECT_EQ(detections.front().box, cv::Rect(40, 45, 20, 30));
}
