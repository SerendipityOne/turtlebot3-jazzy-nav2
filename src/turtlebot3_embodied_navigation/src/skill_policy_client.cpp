// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/skill_policy_client.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace turtlebot3_embodied_navigation
{

namespace
{

constexpr std::size_t kMaximumJpegBytes = 2U * 1024U * 1024U;
constexpr double kMaximumLocalTranslationM = 0.75;
constexpr double kMaximumLocalYawRad = 0.78539816339744830962;

size_t write_response(char * data, size_t size, size_t count, void * user_data)
{
  const size_t bytes = size * count;
  static_cast<std::string *>(user_data)->append(data, bytes);
  return bytes;
}

std::string encode_base64(const std::vector<uint8_t> & input)
{
  static constexpr char alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve((input.size() + 2) / 3 * 4);
  for (std::size_t index = 0; index < input.size(); index += 3) {
    const uint32_t first = input[index];
    const uint32_t second = index + 1 < input.size() ? input[index + 1] : 0U;
    const uint32_t third = index + 2 < input.size() ? input[index + 2] : 0U;
    const uint32_t value = (first << 16U) | (second << 8U) | third;
    output.push_back(alphabet[(value >> 18U) & 0x3FU]);
    output.push_back(alphabet[(value >> 12U) & 0x3FU]);
    output.push_back(index + 1 < input.size() ? alphabet[(value >> 6U) & 0x3FU] : '=');
    output.push_back(index + 2 < input.size() ? alphabet[value & 0x3FU] : '=');
  }
  return output;
}

bool is_loopback_endpoint(const std::string & endpoint)
{
  return endpoint.rfind("http://127.0.0.1:", 0) == 0 ||
         endpoint.rfind("http://localhost:", 0) == 0;
}

std::string post_policy_request(
  const std::string & endpoint, const long timeout_milliseconds,
  const nlohmann::json & request)
{
  std::string response;
  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    throw std::runtime_error("curl_easy_init failed");
  }
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  const std::string payload = request.dump();
  curl_easy_setopt(curl, CURLOPT_URL, endpoint.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_milliseconds);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  const CURLcode result = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  if (result != CURLE_OK) {
    throw std::runtime_error(curl_easy_strerror(result));
  }
  if (status < 200 || status >= 300) {
    throw std::runtime_error("skill policy returned HTTP " + std::to_string(status));
  }
  return response;
}

void validate_policy_request(
  const std::string & instruction, const std::vector<uint8_t> & jpeg,
  const std::vector<double> & robot_state, const std::size_t expected_state_size)
{
  if (instruction.empty() || jpeg.empty() || jpeg.size() > kMaximumJpegBytes ||
    robot_state.size() != expected_state_size)
  {
    throw std::invalid_argument("policy request requires instruction, image and robot state");
  }
  if (!std::all_of(robot_state.begin(), robot_state.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument("policy state contains a non-finite value");
  }
}

}  // namespace

VLAActionDecision parse_vla_policy_response(const std::string & response)
{
  const auto json = nlohmann::json::parse(response);
  if (json.at("schema_version").get<int>() != 1) {
    throw std::invalid_argument("policy response schema version is unsupported");
  }
  const int action_id = json.at("action_id").get<int>();
  const float prototype_distance = json.at("prototype_distance").get<float>();
  const float prototype_gap = json.at("prototype_gap").get<float>();
  if (action_id < 0 || action_id >= static_cast<int>(kVlaActionCount)) {
    throw std::invalid_argument("policy returned an unknown action id");
  }
  if (!std::isfinite(prototype_distance) || !std::isfinite(prototype_gap) ||
    prototype_distance < 0.0F || prototype_gap < 0.0F)
  {
    throw std::invalid_argument("policy prototype metrics must be finite and non-negative");
  }
  const std::string model_version = json.at("model_version").get<std::string>();
  if (model_version.empty()) {
    throw std::invalid_argument("policy model version must not be empty");
  }
  return VLAActionDecision{
    static_cast<VLAAction>(action_id), prototype_distance, prototype_gap,
    model_version};
}

ContinuousVLAActionDecision parse_continuous_vla_policy_response(const std::string & response)
{
  const auto json = nlohmann::json::parse(response);
  if (json.at("schema_version").get<int>() != 2) {
    throw std::invalid_argument("continuous policy response schema version is unsupported");
  }
  ContinuousVLAAction action{
    json.at("delta_x_m").get<double>(),
    json.at("delta_y_m").get<double>(),
    json.at("delta_yaw_rad").get<double>()};
  const double translation = std::hypot(action.delta_x_m, action.delta_y_m);
  if (!std::isfinite(action.delta_x_m) || !std::isfinite(action.delta_y_m) ||
    !std::isfinite(action.delta_yaw_rad) || translation > kMaximumLocalTranslationM ||
    std::abs(action.delta_yaw_rad) > kMaximumLocalYawRad)
  {
    throw std::invalid_argument("continuous policy action is outside the local safety bounds");
  }
  const std::string model_version = json.at("model_version").get<std::string>();
  const float inference_milliseconds = json.at("inference_milliseconds").get<float>();
  if (model_version.empty() || !std::isfinite(inference_milliseconds) || inference_milliseconds < 0.0F) {
    throw std::invalid_argument("continuous policy metadata is invalid");
  }
  return ContinuousVLAActionDecision{action, model_version, inference_milliseconds};
}

std::string vla_action_name(const VLAAction action)
{
  switch (action) {
    case VLAAction::GO_TO_CANDIDATE_0: return "go_to_candidate_0";
    case VLAAction::GO_TO_CANDIDATE_1: return "go_to_candidate_1";
    case VLAAction::GO_TO_CANDIDATE_2: return "go_to_candidate_2";
    case VLAAction::SCAN_LEFT: return "scan_left";
    case VLAAction::SCAN_RIGHT: return "scan_right";
    case VLAAction::APPROACH_RED: return "approach_red";
    case VLAAction::APPROACH_GREEN: return "approach_green";
    case VLAAction::APPROACH_BLUE: return "approach_blue";
    case VLAAction::APPROACH_YELLOW: return "approach_yellow";
    case VLAAction::REPORT_NOT_FOUND: return "report_not_found";
  }
  throw std::invalid_argument("unknown VLA action");
}

std::optional<std::string> vla_approach_color(const VLAAction action)
{
  switch (action) {
    case VLAAction::APPROACH_RED: return "red";
    case VLAAction::APPROACH_GREEN: return "green";
    case VLAAction::APPROACH_BLUE: return "blue";
    case VLAAction::APPROACH_YELLOW: return "yellow";
    default: return std::nullopt;
  }
}

bool is_vla_candidate_action(const VLAAction action)
{
  return action == VLAAction::GO_TO_CANDIDATE_0 ||
         action == VLAAction::GO_TO_CANDIDATE_1 ||
         action == VLAAction::GO_TO_CANDIDATE_2;
}

std::size_t vla_candidate_slot(const VLAAction action)
{
  if (!is_vla_candidate_action(action)) {
    throw std::invalid_argument("VLA action does not select a candidate slot");
  }
  return static_cast<std::size_t>(action);
}

VLAActionPolicyClient::VLAActionPolicyClient(
  std::string endpoint, const long timeout_milliseconds)
: endpoint_(std::move(endpoint)), timeout_milliseconds_(timeout_milliseconds)
{
  if (!is_loopback_endpoint(endpoint_)) {
    throw std::invalid_argument("skill policy endpoint must use loopback HTTP");
  }
  if (timeout_milliseconds_ <= 0) {
    throw std::invalid_argument("skill policy timeout must be positive");
  }
}

VLAActionDecision VLAActionPolicyClient::select(
  const std::string & instruction, const std::vector<uint8_t> & jpeg,
  const std::vector<double> & robot_state) const
{
  validate_policy_request(instruction, jpeg, robot_state, kVlaPolicyStateSize);
  const nlohmann::json request = {
    {"schema_version", 1},
    {"instruction", instruction},
    {"image_jpeg_base64", encode_base64(jpeg)},
    {"robot_state", robot_state}};
  return parse_vla_policy_response(post_policy_request(endpoint_, timeout_milliseconds_, request));
}

ContinuousVLAActionPolicyClient::ContinuousVLAActionPolicyClient(
  std::string endpoint, const long timeout_milliseconds)
: endpoint_(std::move(endpoint)), timeout_milliseconds_(timeout_milliseconds)
{
  if (!is_loopback_endpoint(endpoint_)) {
    throw std::invalid_argument("continuous policy endpoint must use loopback HTTP");
  }
  if (timeout_milliseconds_ <= 0) {
    throw std::invalid_argument("continuous policy timeout must be positive");
  }
}

ContinuousVLAActionDecision ContinuousVLAActionPolicyClient::select(
  const std::string & stage_instruction, const std::vector<uint8_t> & jpeg,
  const std::vector<double> & robot_state) const
{
  validate_policy_request(
    stage_instruction, jpeg, robot_state, kContinuousVlaPolicyStateSize);
  const nlohmann::json request = {
    {"schema_version", 2},
    {"instruction", stage_instruction},
    {"image_jpeg_base64", encode_base64(jpeg)},
    {"robot_state", robot_state}};
  return parse_continuous_vla_policy_response(
    post_policy_request(endpoint_, timeout_milliseconds_, request));
}

}  // namespace turtlebot3_embodied_navigation
