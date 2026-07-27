// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/instruction_parser.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>
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

struct HttpResponse
{
  std::string body;
  std::string content_type;
};

bool equals_ignore_case(const std::string & value, const std::string & expected)
{
  return value.size() == expected.size() && std::equal(
    value.begin(), value.end(), expected.begin(),
    [](unsigned char left, unsigned char right) {
      return std::tolower(left) == std::tolower(right);
    });
}

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool is_json_content_type(const std::string & content_type)
{
  std::string normalized = content_type;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return normalized.empty() || normalized.find("json") != std::string::npos;
}

bool is_event_stream_content_type(const std::string & content_type)
{
  std::string normalized = content_type;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return normalized.find("text/event-stream") != std::string::npos;
}

size_t write_response(char * data, size_t size, size_t count, void * user_data)
{
  const size_t bytes = size * count;
  static_cast<HttpResponse *>(user_data)->body.append(data, bytes);
  return bytes;
}

size_t write_header(char * data, size_t size, size_t count, void * user_data)
{
  const size_t bytes = size * count;
  const std::string header(data, bytes);
  const auto separator = header.find(':');
  if (separator != std::string::npos &&
    equals_ignore_case(header.substr(0, separator), "Content-Type"))
  {
    static_cast<HttpResponse *>(user_data)->content_type = trim(header.substr(separator + 1));
  }
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

std::string extract_completed_response_output_text(const nlohmann::json & body)
{
  if (!body.is_object()) {
    throw std::runtime_error("Responses API returned an invalid JSON envelope");
  }
  if (body.value("status", "") != "completed") {
    throw std::runtime_error("Responses API response did not complete");
  }

  const auto output = body.find("output");
  if (output == body.end() || !output->is_array()) {
    throw std::runtime_error("Responses API response has no output array");
  }
  // 原始 REST 响应没有顶层 output_text，需从 message.content 中逐项读取。
  for (const auto & item : *output) {
    if (!item.is_object() || item.value("type", "") != "message") {
      continue;
    }
    const auto content = item.find("content");
    if (content == item.end() || !content->is_array()) {
      continue;
    }
    for (const auto & part : *content) {
      if (!part.is_object()) {
        continue;
      }
      if (part.value("type", "") == "refusal") {
        throw std::runtime_error("Responses API returned a refusal");
      }
      if (part.value("type", "") == "output_text" &&
        part.contains("text") && part.at("text").is_string())
      {
        return part.at("text").get<std::string>();
      }
    }
  }
  throw std::runtime_error("Responses API response has no output_text");
}

std::string extract_sse_output_text(const std::string & response_body)
{
  bool saw_event = false;
  bool is_completed = false;
  std::string delta_text;
  std::string completed_text;
  std::string response_text;

  const auto process_event =
    [&](const std::string & event_data) {
      if (event_data.empty() || event_data == "[DONE]") {
        return;
      }

      nlohmann::json event;
      try {
        event = nlohmann::json::parse(event_data);
      } catch (const nlohmann::json::exception &) {
        throw std::runtime_error("Responses API returned an invalid SSE event");
      }
      if (!event.is_object()) {
        throw std::runtime_error("Responses API returned an invalid SSE event");
      }
      saw_event = true;
      const std::string type = event.value("type", "");
      if (type == "response.output_text.delta" && event.contains("delta") &&
        event.at("delta").is_string())
      {
        delta_text += event.at("delta").get<std::string>();
      } else if (type == "response.output_text.done" && event.contains("text") &&
        event.at("text").is_string())
      {
        completed_text = event.at("text").get<std::string>();
      } else if (type.rfind("response.refusal.", 0) == 0) {
        throw std::runtime_error("Responses API returned a refusal");
      } else if (type == "response.failed" || type == "response.incomplete" ||
        type == "error")
      {
        throw std::runtime_error("Responses API stream failed");
      } else if (type == "response.completed") {
        const auto response = event.find("response");
        if (response == event.end() || !response->is_object() ||
          response->value("status", "") != "completed")
        {
          throw std::runtime_error("Responses API stream did not complete");
        }
        is_completed = true;
        const auto output = response->find("output");
        if (output != response->end() && output->is_array() && !output->empty()) {
          response_text = extract_completed_response_output_text(*response);
        }
      }
    };

  std::istringstream stream(response_body);
  std::string line;
  std::string event_data;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      process_event(event_data);
      event_data.clear();
      continue;
    }
    if (line.rfind("data:", 0) != 0) {
      continue;
    }
    std::string data = line.substr(5);
    if (!data.empty() && data.front() == ' ') {
      data.erase(0, 1);
    }
    if (!event_data.empty()) {
      event_data += '\n';
    }
    event_data += data;
  }
  process_event(event_data);

  if (!saw_event) {
    throw std::runtime_error("Responses API returned a non-JSON response");
  }
  if (!is_completed) {
    throw std::runtime_error("Responses API stream did not complete");
  }
  if (!completed_text.empty()) {
    return completed_text;
  }
  if (!response_text.empty()) {
    return response_text;
  }
  if (!delta_text.empty()) {
    return delta_text;
  }
  throw std::runtime_error("Responses API response has no output_text");
}

}  // namespace

std::string build_responses_request(
  const std::string & model,
  const std::string & instruction,
  const std::vector<std::string> & allowed_classes,
  const std::vector<std::string> & allowed_colors,
  const std::vector<std::string> & allowed_rooms)
{
  const std::string system_prompt =
    "Extract an object-search request. Return only one JSON object with string fields "
    "object_class, color, room. Allowed object_class: " + join(allowed_classes) +
    ". Allowed color: " + join(allowed_colors) + ". Allowed room: " +
    join(allowed_rooms) + ". Do not add fields or prose.";
  const nlohmann::json schema = {
    {"type", "object"},
    {"properties", {
      {"object_class", {{"type", "string"}, {"enum", allowed_classes}}},
      {"color", {{"type", "string"}, {"enum", allowed_colors}}},
      {"room", {{"type", "string"}, {"enum", allowed_rooms}}}}},
    {"required", nlohmann::json::array({"object_class", "color", "room"})},
    {"additionalProperties", false}};
  const nlohmann::json request = {
    {"model", model},
    {"instructions", system_prompt},
    {"input", instruction},
    {"store", false},
    {"stream", false},
    {"text", {{"format", {
      {"type", "json_schema"},
      {"name", "find_object_target"},
      {"strict", true},
      {"schema", schema}}}}}};
  return request.dump();
}

std::string extract_responses_output_text(const std::string & response_body)
{
  try {
    const auto body = nlohmann::json::parse(response_body);
    return extract_completed_response_output_text(body);
  } catch (const nlohmann::json::parse_error &) {
    return extract_sse_output_text(response_body);
  } catch (const nlohmann::json::exception &) {
    throw std::runtime_error("Responses API returned an invalid JSON envelope");
  }
}

TargetSpec parse_target_json(
  const std::string & response_body,
  const std::vector<std::string> & allowed_classes,
  const std::vector<std::string> & allowed_colors,
  const std::vector<std::string> & allowed_rooms)
{
  TargetSpec target;
  try {
    const auto json = nlohmann::json::parse(response_body);
    target = TargetSpec{
      json.at("object_class").get<std::string>(),
      json.at("color").get<std::string>(),
      json.at("room").get<std::string>()};
  } catch (const nlohmann::json::exception &) {
    throw std::invalid_argument("Responses API output_text is not valid target JSON");
  }

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

  const std::string payload = build_responses_request(
    model_, instruction, allowed_classes_, allowed_colors_, allowed_rooms_);

  std::string last_error;
  for (int attempt = 0; attempt < 3; ++attempt) {
    CURL * curl = curl_easy_init();
    if (curl == nullptr) {
      throw std::runtime_error("curl_easy_init failed");
    }
    HttpResponse response;
    struct curl_slist * headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key_).c_str());
    const std::string endpoint = api_base_ + "/responses";
    curl_easy_setopt(curl, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, write_header);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &response);

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
      if (!is_json_content_type(response.content_type) &&
        !is_event_stream_content_type(response.content_type))
      {
        throw std::runtime_error(
          "Responses API returned unsupported content (HTTP " + std::to_string(status) +
          ", Content-Type: " + response.content_type + ")");
      }
      return parse_target_json(
        extract_responses_output_text(response.body), allowed_classes_, allowed_colors_, allowed_rooms_);
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
