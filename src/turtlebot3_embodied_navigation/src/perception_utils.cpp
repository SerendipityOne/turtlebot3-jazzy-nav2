// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/perception_utils.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace turtlebot3_embodied_navigation
{

namespace
{

cv::Rect clamp_box(const cv::Rect & box, const cv::Size & size)
{
  return box & cv::Rect(0, 0, size.width, size.height);
}

}  // namespace

std::vector<Detection2D> decode_yolox(
  const cv::Mat & output, const double resize_scale, const cv::Size & image_size,
  const int input_size, const int class_count, const float confidence_threshold,
  const float nms_threshold)
{
  if (output.empty() || resize_scale <= 0.0 || input_size <= 0 || class_count <= 0) {
    throw std::invalid_argument("invalid YOLOX decoder input");
  }

  const int dimensions = class_count + 5;
  const int rows = static_cast<int>(output.total() / dimensions);
  if (rows <= 0 || output.total() % dimensions != 0) {
    throw std::runtime_error("unexpected YOLOX output shape");
  }

  const cv::Mat predictions(rows, dimensions, CV_32F, const_cast<float *>(output.ptr<float>()));
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_indices;

  for (int row = 0; row < rows; ++row) {
    const float * values = predictions.ptr<float>(row);
    const float objectness = values[4];
    int best_class = 0;
    float best_class_score = values[5];
    for (int class_index = 1; class_index < class_count; ++class_index) {
      if (values[5 + class_index] > best_class_score) {
        best_class = class_index;
        best_class_score = values[5 + class_index];
      }
    }

    const float confidence = objectness * best_class_score;
    if (confidence < confidence_threshold) {
      continue;
    }

    const double center_x = values[0] / resize_scale;
    const double center_y = values[1] / resize_scale;
    const double width = values[2] / resize_scale;
    const double height = values[3] / resize_scale;
    cv::Rect box(
      cvRound(center_x - width * 0.5), cvRound(center_y - height * 0.5),
      cvRound(width), cvRound(height));
    box = clamp_box(box, image_size);
    if (box.area() <= 0) {
      continue;
    }
    boxes.push_back(box);
    scores.push_back(confidence);
    class_indices.push_back(best_class);
  }

  std::vector<int> selected;
  cv::dnn::NMSBoxes(boxes, scores, confidence_threshold, nms_threshold, selected);
  std::vector<Detection2D> detections;
  detections.reserve(selected.size());
  for (const int index : selected) {
    detections.push_back({boxes[index], class_indices[index], scores[index]});
  }
  return detections;
}

float median_depth(
  const cv::Mat & depth, const cv::Rect & box, const float minimum_depth,
  const float maximum_depth)
{
  if (depth.type() != CV_32FC1 || minimum_depth <= 0.0F || maximum_depth <= minimum_depth) {
    throw std::invalid_argument("depth must be CV_32FC1 with a valid range");
  }

  const cv::Rect clipped = clamp_box(box, depth.size());
  if (clipped.area() <= 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const int margin_x = clipped.width / 4;
  const int margin_y = clipped.height / 4;
  const cv::Rect center(
    clipped.x + margin_x, clipped.y + margin_y,
    std::max(1, clipped.width - 2 * margin_x),
    std::max(1, clipped.height - 2 * margin_y));

  std::vector<float> valid_depths;
  valid_depths.reserve(center.area());
  for (int row = center.y; row < center.y + center.height; ++row) {
    const float * values = depth.ptr<float>(row);
    for (int column = center.x; column < center.x + center.width; ++column) {
      const float value = values[column];
      if (std::isfinite(value) && value >= minimum_depth && value <= maximum_depth) {
        valid_depths.push_back(value);
      }
    }
  }

  if (valid_depths.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto middle = valid_depths.begin() + valid_depths.size() / 2;
  std::nth_element(valid_depths.begin(), middle, valid_depths.end());
  return *middle;
}

Point3D project_pixel(
  const double pixel_x, const double pixel_y, const double depth,
  const double fx, const double fy, const double cx, const double cy)
{
  if (!std::isfinite(depth) || depth <= 0.0 || fx <= 0.0 || fy <= 0.0) {
    throw std::invalid_argument("invalid depth or camera intrinsics");
  }
  return {(pixel_x - cx) * depth / fx, (pixel_y - cy) * depth / fy, depth};
}

std::string classify_color(const cv::Mat & bgr_image, const cv::Rect & box)
{
  const cv::Rect clipped = clamp_box(box, bgr_image.size());
  if (bgr_image.type() != CV_8UC3 || clipped.area() <= 0) {
    return "unknown";
  }

  cv::Mat hsv;
  cv::cvtColor(bgr_image(clipped), hsv, cv::COLOR_BGR2HSV);
  cv::Scalar mean = cv::mean(hsv);
  const double hue = mean[0];
  const double saturation = mean[1];
  if (saturation < 45.0) {
    return "unknown";
  }
  if (hue < 10.0 || hue >= 170.0) {
    return "red";
  }
  if (hue < 38.0) {
    return "yellow";
  }
  if (hue < 85.0) {
    return "green";
  }
  if (hue < 135.0) {
    return "blue";
  }
  return "unknown";
}

}  // namespace turtlebot3_embodied_navigation
