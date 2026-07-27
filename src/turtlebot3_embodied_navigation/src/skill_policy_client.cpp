// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/skill_policy_client.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace turtlebot3_embodied_navigation
{

namespace
{

constexpr std::array<const char *, 5> kAllowedSkills{
  "go_to_viewpoint", "rotate_scan", "approach_target", "report_not_found", "stop"};

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

}  // namespace

SkillDecision parse_skill_response(const std::string & response)
{
  const auto json = nlohmann::json::parse(response);
  SkillDecision decision{
    json.at("skill").get<std::string>(), json.at("confidence").get<float>()};
  const bool allowed = std::any_of(
    kAllowedSkills.begin(), kAllowedSkills.end(),
    [&decision](const char * skill) {return decision.skill == skill;});
  if (!allowed) {
    throw std::invalid_argument("policy returned an unknown skill");
  }
  if (decision.confidence < 0.0F || decision.confidence > 1.0F) {
    throw std::invalid_argument("policy confidence must be within [0, 1]");
  }
  return decision;
}

SkillPolicyClient::SkillPolicyClient(std::string endpoint, const long timeout_milliseconds)
: endpoint_(std::move(endpoint)), timeout_milliseconds_(timeout_milliseconds)
{
  if (!is_loopback_endpoint(endpoint_)) {
    throw std::invalid_argument("skill policy endpoint must use loopback HTTP");
  }
  if (timeout_milliseconds_ <= 0) {
    throw std::invalid_argument("skill policy timeout must be positive");
  }
}

SkillDecision SkillPolicyClient::select(
  const std::string & instruction, const std::vector<uint8_t> & jpeg,
  const std::vector<double> & robot_state) const
{
  if (instruction.empty() || jpeg.empty() || robot_state.empty()) {
    throw std::invalid_argument("policy request requires instruction, image and robot state");
  }
  const nlohmann::json request = {
    {"instruction", instruction},
    {"image_jpeg_base64", encode_base64(jpeg)},
    {"robot_state", robot_state}};
  const std::string payload = request.dump();
  std::string response;

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    throw std::runtime_error("curl_easy_init failed");
  }
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, endpoint_.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_milliseconds_);
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
  return parse_skill_response(response);
}

}  // namespace turtlebot3_embodied_navigation
