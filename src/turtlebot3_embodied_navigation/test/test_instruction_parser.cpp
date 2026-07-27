// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

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

TEST(InstructionParser, BuildsStrictResponsesRequest)
{
  const auto request = nlohmann::json::parse(
    turtlebot3_embodied_navigation::build_responses_request(
      "gpt-4.1-mini", "find a red cup", classes, colors, rooms));

  EXPECT_EQ(request.at("model"), "gpt-4.1-mini");
  EXPECT_EQ(request.at("input"), "find a red cup");
  EXPECT_FALSE(request.at("store"));
  EXPECT_FALSE(request.at("stream"));
  const auto & format = request.at("text").at("format");
  EXPECT_EQ(format.at("type"), "json_schema");
  EXPECT_EQ(format.at("name"), "find_object_target");
  EXPECT_TRUE(format.at("strict"));
  const auto & schema = format.at("schema");
  EXPECT_EQ(schema.at("required"), nlohmann::json::array({"object_class", "color", "room"}));
  EXPECT_FALSE(schema.at("additionalProperties"));
  EXPECT_EQ(schema.at("properties").at("object_class").at("enum"), classes);
  EXPECT_EQ(schema.at("properties").at("color").at("enum"), colors);
  EXPECT_EQ(schema.at("properties").at("room").at("enum"), rooms);
}

TEST(InstructionParser, ExtractsCompletedResponsesOutputText)
{
  const auto text = turtlebot3_embodied_navigation::extract_responses_output_text(
    R"({"id":"resp_123","object":"response","created_at":0,"model":"gpt-4.1-mini","status":"completed","output":[{"id":"msg_123","type":"message","status":"completed","role":"assistant","content":[{"type":"output_text","annotations":[],"text":"{\"object_class\":\"cup\",\"color\":\"red\",\"room\":\"kitchen\"}"}]}]})");

  EXPECT_EQ(text, R"({"object_class":"cup","color":"red","room":"kitchen"})");
}

TEST(InstructionParser, ExtractsCompletedStreamingResponsesOutputText)
{
  const auto text = turtlebot3_embodied_navigation::extract_responses_output_text(
    "event: response.output_text.delta\n"
    "data: {\"type\":\"response.output_text.delta\",\"delta\":\"{\\\"object_class\\\":\\\"cup\\\",\"}\n\n"
    "event: response.output_text.done\n"
    "data: {\"type\":\"response.output_text.done\",\"text\":\"{\\\"object_class\\\":\\\"cup\\\",\\\"color\\\":\\\"red\\\",\\\"room\\\":\\\"kitchen\\\"}\"}\n\n"
    "event: response.completed\n"
    "data: {\"type\":\"response.completed\",\"response\":{\"status\":\"completed\",\"output\":[]}}\n\n");

  EXPECT_EQ(text, R"({"object_class":"cup","color":"red","room":"kitchen"})");
}

TEST(InstructionParser, ExtractsOutputTextFromCompletedStreamingResponseObject)
{
  const auto text = turtlebot3_embodied_navigation::extract_responses_output_text(
    "event: response.completed\n"
    "data: {\"type\":\"response.completed\",\"response\":{\"status\":\"completed\",\"output\":[{\"type\":\"message\",\"content\":[{\"type\":\"output_text\",\"text\":\"{\\\"object_class\\\":\\\"cup\\\",\\\"color\\\":\\\"red\\\",\\\"room\\\":\\\"kitchen\\\"}\"}]}]}}\n\n");

  EXPECT_EQ(text, R"({"object_class":"cup","color":"red","room":"kitchen"})");
}

TEST(InstructionParser, RejectsIncompleteOrRefusedStreamingResponses)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::extract_responses_output_text(
      "data: {\"type\":\"response.output_text.delta\",\"delta\":\"partial\"}\n\n"),
    std::runtime_error);
  EXPECT_THROW(
    turtlebot3_embodied_navigation::extract_responses_output_text(
      "data: {\"type\":\"response.refusal.done\",\"refusal\":\"no\"}\n\n"),
    std::runtime_error);
  EXPECT_THROW(
    turtlebot3_embodied_navigation::extract_responses_output_text(
      "data: {\"type\":\"response.failed\",\"response\":{\"status\":\"failed\"}}\n\n"),
    std::runtime_error);
}

TEST(InstructionParser, RejectsIncompleteOrRefusedResponses)
{
  EXPECT_THROW(
    turtlebot3_embodied_navigation::extract_responses_output_text(
      R"({"status":"incomplete","output":[]})"),
    std::runtime_error);
  EXPECT_THROW(
    turtlebot3_embodied_navigation::extract_responses_output_text(
      R"({"status":"completed","output":[{"type":"message","content":[{"type":"refusal","refusal":"no"}]}]})"),
    std::runtime_error);
  EXPECT_THROW(
    turtlebot3_embodied_navigation::extract_responses_output_text(
      R"({"status":"completed","output":[]})"),
    std::runtime_error);
}

TEST(InstructionParser, ReportsNonJsonResponsesWithoutLeakingTheirBody)
{
  try {
    turtlebot3_embodied_navigation::extract_responses_output_text("error");
    FAIL() << "Expected a non-JSON response to be rejected";
  } catch (const std::runtime_error & error) {
    EXPECT_STREQ(error.what(), "Responses API returned a non-JSON response");
  }
}

TEST(InstructionParser, ReportsInvalidTargetOutputText)
{
  const auto output_text = turtlebot3_embodied_navigation::extract_responses_output_text(
    R"({"status":"completed","output":[{"type":"message","content":[{"type":"output_text","text":"error"}]}]})");

  try {
    turtlebot3_embodied_navigation::parse_target_json(output_text, classes, colors, rooms);
    FAIL() << "Expected invalid target JSON to be rejected";
  } catch (const std::invalid_argument & error) {
    EXPECT_STREQ(error.what(), "Responses API output_text is not valid target JSON");
  }
}

}  // namespace
