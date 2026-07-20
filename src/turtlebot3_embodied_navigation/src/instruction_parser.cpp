// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/instruction_parser.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace turtlebot3_embodied_navigation
{

namespace
{

bool contains(const std::vector<std::string> & values, const std::string & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

size_t write_response(char * data, size_t size, size_t count, void * user_data)
{
  const size_t bytes = size * count;
  static_cast<std::string *>(user_data)->append(data, bytes);
  return bytes;
}

std::string join(const std::vector<std::string> & values)
{
  std::string result;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      result += ", ";
    }
    result += values[index];
  }
  return result;
}

std::string extract_content(const std::string & response)
{
  const auto body = nlohmann::json::parse(response);
  return body.at("choices").at(0).at("message").at("content").get<std::string>();
}

}  // namespace

TargetSpec parse_target_json(
  const std::string & response_body,
  const std::vector<std::string> & allowed_classes,
  const std::vector<std::string> & allowed_colors,
  const std::vector<std::string> & allowed_rooms)
{
  const auto json = nlohmann::json::parse(response_body);
  TargetSpec target{
    json.at("object_class").get<std::string>(),
    json.at("color").get<std::string>(),
    json.at("room").get<std::string>()};

  if (!contains(allowed_classes, target.object_class)) {
    throw std::invalid_argument("object_class is outside the configured whitelist");
  }
  if (!contains(allowed_colors, target.color)) {
    throw std::invalid_argument("color is outside the configured whitelist");
  }
  if (!contains(allowed_rooms, target.room)) {
    throw std::invalid_argument("room is outside the configured whitelist");
  }
  return target;
}

OpenAiInstructionParser::OpenAiInstructionParser(
  std::string api_base, std::string model, std::string api_key,
  std::vector<std::string> allowed_classes,
  std::vector<std::string> allowed_colors,
  std::vector<std::string> allowed_rooms)
: api_base_(std::move(api_base)),
  model_(std::move(model)),
  api_key_(std::move(api_key)),
  allowed_classes_(std::move(allowed_classes)),
  allowed_colors_(std::move(allowed_colors)),
  allowed_rooms_(std::move(allowed_rooms))
{
  if (api_base_.empty() || model_.empty() || api_key_.empty()) {
    throw std::invalid_argument("VLM_API_BASE, VLM_MODEL and VLM_API_KEY are required");
  }
}

TargetSpec OpenAiInstructionParser::parse(const std::string & instruction) const
{
  if (instruction.empty()) {
    throw std::invalid_argument("instruction must not be empty");
  }

  const std::string system_prompt =
    "Extract an object-search request. Return only one JSON object with string fields "
    "object_class, color, room. Allowed object_class: " + join(allowed_classes_) +
    ". Allowed color: " + join(allowed_colors_) + ". Allowed room: " +
    join(allowed_rooms_) + ". Do not add fields or prose.";
  const nlohmann::json request = {
    {"model", model_},
    {"temperature", 0},
    {"messages", nlohmann::json::array({
      {{"role", "system"}, {"content", system_prompt}},
      {{"role", "user"}, {"content", instruction}}})}};

  std::string last_error;
  for (int attempt = 0; attempt < 3; ++attempt) {
    CURL * curl = curl_easy_init();
    if (curl == nullptr) {
      throw std::runtime_error("curl_easy_init failed");
    }
    std::string response;
    struct curl_slist * headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key_).c_str());
    const std::string endpoint = api_base_ + "/chat/completions";
    const std::string payload = request.dump();
    curl_easy_setopt(curl, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    const CURLcode result = curl_easy_perform(curl);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    try {
      if (result != CURLE_OK) {
        throw std::runtime_error(curl_easy_strerror(result));
      }
      if (status < 200 || status >= 300) {
        throw std::runtime_error("language API returned HTTP " + std::to_string(status));
      }
      return parse_target_json(
        extract_content(response), allowed_classes_, allowed_colors_, allowed_rooms_);
    } catch (const std::exception & error) {
      last_error = error.what();
    }

    if (attempt < 2) {
      std::this_thread::sleep_for(std::chrono::seconds(1 << attempt));
    }
  }
  throw std::runtime_error("language API failed after 3 attempts: " + last_error);
}

}  // namespace turtlebot3_embodied_navigation
