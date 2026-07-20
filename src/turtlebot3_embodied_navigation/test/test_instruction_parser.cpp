// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "turtlebot3_embodied_navigation/instruction_parser.hpp"

namespace
{

const std::vector<std::string> classes{"cup", "bottle"};
const std::vector<std::string> colors{"red", "blue"};
const std::vector<std::string> rooms{"kitchen", "living_room", "unknown"};

TEST(InstructionParser, ParsesWhitelistedTarget)
{
  const auto target = turtlebot3_embodied_navigation::parse_target_json(
    R"({"object_class":"cup","color":"red","room":"kitchen"})",
    classes, colors, rooms);
  EXPECT_EQ(target.object_class, "cup");
  EXPECT_EQ(target.color, "red");
  EXPECT_EQ(target.room, "kitchen");
}

TEST(InstructionParser, RejectsUnknownObjectClass)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_target_json(
      R"({"object_class":"person","color":"red","room":"kitchen"})",
      classes, colors, rooms),
    std::invalid_argument);
}

TEST(InstructionParser, RejectsMissingFields)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::parse_target_json(
      R"({"object_class":"cup","color":"red"})", classes, colors, rooms),
    std::exception);
}

}  // namespace
