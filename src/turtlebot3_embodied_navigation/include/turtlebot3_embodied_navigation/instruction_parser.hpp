// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#ifndef TURTLEBOT3_EMBODIED_NAVIGATION__INSTRUCTION_PARSER_HPP_
#define TURTLEBOT3_EMBODIED_NAVIGATION__INSTRUCTION_PARSER_HPP_

#include <string>
#include <vector>

namespace turtlebot3_embodied_navigation
{

struct TargetSpec
{
  std::string object_class;
  std::string color;
  std::string room;
};

enum class TaskStage
{
  SEARCH,
  APPROACH,
  VERIFY,
  DONE,
  FAIL,
};

struct StageContext
{
  std::string outcome;
  double coverage_ratio;
  double detection_match_ratio;
  double target_distance_m;
};

struct StagePlan
{
  TaskStage stage;
  std::string stage_instruction;
  std::string reason;
};

// 构造符合 OpenAI Responses API 严格 JSON Schema 约束的请求体。
std::string build_responses_request(
  const std::string & model,
  const std::string & instruction,
  const std::vector<std::string> & allowed_classes,
  const std::vector<std::string> & allowed_colors,
  const std::vector<std::string> & allowed_rooms);

// 从已完成的 Responses API 响应中取得模型输出文本。
std::string extract_responses_output_text(const std::string & response_body);

TargetSpec parse_target_json(
  const std::string & response_body,
  const std::vector<std::string> & allowed_classes,
  const std::vector<std::string> & allowed_colors,
  const std::vector<std::string> & allowed_rooms);

std::string task_stage_name(TaskStage stage);
StagePlan parse_stage_plan_json(
  const std::string & response_body, const TargetSpec & target,
  const std::vector<TaskStage> & allowed_stages);

class OpenAiInstructionParser
{
public:
  OpenAiInstructionParser(
    std::string api_base, std::string model, std::string api_key,
    std::vector<std::string> allowed_classes,
    std::vector<std::string> allowed_colors,
    std::vector<std::string> allowed_rooms);

  TargetSpec parse(const std::string & instruction) const;
  StagePlan plan_next_stage(
    const std::string & instruction, const TargetSpec & target, const StageContext & context,
    const std::vector<TaskStage> & allowed_stages) const;

private:
  std::string request_output(const std::string & payload) const;
  std::string api_base_;
  std::string model_;
  std::string api_key_;
  std::vector<std::string> allowed_classes_;
  std::vector<std::string> allowed_colors_;
  std::vector<std::string> allowed_rooms_;
};

}  // namespace turtlebot3_embodied_navigation

#endif  // TURTLEBOT3_EMBODIED_NAVIGATION__INSTRUCTION_PARSER_HPP_
