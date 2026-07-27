#include <gtest/gtest.h>

#include <cstdint>
#include <array>
#include <string>
#include <vector>

#include "turtlebot3_gazebo/dataset_utils.hpp"

namespace tb3 = turtlebot3_gazebo;

TEST(DatasetUtils, DecodesGazeboPanopticChannelsAndFiltersTinyInstances)
{
  constexpr std::uint32_t width = 20;
  constexpr std::uint32_t height = 20;
  std::vector<std::uint8_t> data(width * height * 3, 0);
  for (std::uint32_t y = 4; y < 12; ++y) {
    for (std::uint32_t x = 3; x < 11; ++x) {
      const auto offset = (y * width + x) * 3;
      data[offset] = 7;      // Low instance byte.
      data[offset + 1] = 1;  // High instance byte.
      data[offset + 2] = 2;  // Gazebo label; YOLO class becomes 1.
    }
  }
  data[0] = 9;
  data[1] = 0;
  data[2] = 3;

  const auto boxes = tb3::decode_instance_boxes(data, width, height, width * 3, 16, 4);
  ASSERT_EQ(boxes.size(), 1U);
  EXPECT_EQ(boxes[0].class_id, 2);
  EXPECT_EQ(boxes[0].instance_id, 263);
  EXPECT_EQ(boxes[0].visible_pixels, 64U);
  EXPECT_EQ(boxes[0].min_x, 3);
  EXPECT_EQ(boxes[0].max_y, 11);
  EXPECT_EQ(tb3::to_yolo_line(boxes[0], width, height),
    "1 0.35000000 0.40000000 0.40000000 0.40000000");
}

TEST(DatasetUtils, RejectsMalformedLabelMap)
{
  EXPECT_THROW(
    tb3::decode_instance_boxes(std::vector<std::uint8_t>(10), 20, 20, 60, 1, 1),
    std::invalid_argument);
}

TEST(DatasetUtils, DetectsOnlyBoxesOverMaximumAreaRatio)
{
  const tb3::InstanceBox large_box{1, 1, 324, 1, 1, 18, 18};

  EXPECT_TRUE(tb3::is_box_oversized(large_box, 20, 20, 0.75));
  EXPECT_FALSE(tb3::is_box_oversized(large_box, 20, 20, 0.81));
  EXPECT_THROW(tb3::is_box_oversized(large_box, 20, 20, 0.0), std::invalid_argument);
}

TEST(DatasetUtils, CalculatesIntersectionOverUnion)
{
  EXPECT_DOUBLE_EQ(
    tb3::intersection_over_union({0.0, 0.0, 10.0, 10.0}, {0.0, 0.0, 10.0, 10.0}), 1.0);
  EXPECT_DOUBLE_EQ(
    tb3::intersection_over_union({0.0, 0.0, 10.0, 10.0}, {20.0, 20.0, 5.0, 5.0}), 0.0);
  EXPECT_NEAR(
    tb3::intersection_over_union({0.0, 0.0, 10.0, 10.0}, {5.0, 5.0, 10.0, 10.0}),
    25.0 / 175.0, 1e-12);
  EXPECT_THROW(tb3::intersection_over_union({0.0, 0.0, 0.0, 10.0}, {0.0, 0.0, 1.0, 1.0}),
    std::invalid_argument);
}

TEST(DatasetUtils, SelectsCandidatesInDeterministicRoundRobinOrder)
{
  EXPECT_EQ(tb3::round_robin_index(0, 3), 0U);
  EXPECT_EQ(tb3::round_robin_index(1, 3), 1U);
  EXPECT_EQ(tb3::round_robin_index(2, 3), 2U);
  EXPECT_EQ(tb3::round_robin_index(3, 3), 0U);
  EXPECT_THROW(tb3::round_robin_index(0, 0), std::invalid_argument);
}

TEST(DatasetUtils, AppliesSevenOfFifteenAcceptanceGate)
{
  const std::vector<std::string> enabled{"cup", "backpack", "ball", "box"};
  std::vector<tb3::AcceptanceFrame> frames(15);
  for (std::size_t index = 0; index < 7; ++index) {
    frames[index] = {{"cup"}, true};
  }
  EXPECT_TRUE(tb3::evaluate_acceptance_case(frames, "cup", enabled, 7, false).passed);

  for (std::size_t index = 0; index < 7; ++index) {
    frames[index].detected_classes.push_back("backpack");
  }
  EXPECT_FALSE(tb3::evaluate_acceptance_case(frames, "cup", enabled, 7, false).passed);

  frames.assign(15, {});
  frames.front().detected_classes = {"bottle"};
  EXPECT_FALSE(tb3::evaluate_acceptance_case(frames, "", enabled, 7, true).passed);
}

TEST(DatasetUtils, GeneratesDeterministicHardNegativePosesWithinJitter)
{
  const std::vector<std::array<double, 2>> bases{{1.0, 2.0}, {3.0, 4.0}};
  const auto first = tb3::make_hard_negative_pose(0, 20260723, bases, 0.15);
  const auto repeated = tb3::make_hard_negative_pose(0, 20260723, bases, 0.15);
  const auto second = tb3::make_hard_negative_pose(1, 20260723, bases, 0.15);

  EXPECT_DOUBLE_EQ(first.x, repeated.x);
  EXPECT_DOUBLE_EQ(first.y, repeated.y);
  EXPECT_DOUBLE_EQ(first.yaw, repeated.yaw);
  EXPECT_GE(first.x, 0.85);
  EXPECT_LE(first.x, 1.15);
  EXPECT_GE(first.y, 1.85);
  EXPECT_LE(first.y, 2.15);
  EXPECT_GE(second.x, 2.85);
  EXPECT_LE(second.x, 3.15);
  EXPECT_THROW(tb3::make_hard_negative_pose(0, 1, {}, 0.15), std::invalid_argument);
}
