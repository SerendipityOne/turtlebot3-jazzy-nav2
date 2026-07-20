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

std::vector<Detection2D> decode_yolo11(
  const cv::Mat & output, const double resize_scale, const cv::Size & image_size,
  const int padding_x, const int padding_y, const int class_count,
  const float confidence_threshold, const float nms_threshold)
{
  if (output.empty() || output.type() != CV_32F || resize_scale <= 0.0 || class_count <= 0) {
    throw std::invalid_argument("invalid YOLO11 decoder input");
  }

  const int dimensions = class_count + 4;
  int rows = 0;
  bool channels_first = false;
  if (output.dims == 3 && output.size[0] == 1) {
    if (output.size[1] == dimensions) {
      channels_first = true;
      rows = output.size[2];
    } else if (output.size[2] == dimensions) {
      rows = output.size[1];
    }
  } else if (output.dims == 2) {
    if (output.rows == dimensions) {
      channels_first = true;
      rows = output.cols;
    } else if (output.cols == dimensions) {
      rows = output.rows;
    }
  }
  if (rows <= 0 || output.total() != static_cast<std::size_t>(rows * dimensions)) {
    throw std::runtime_error("unexpected YOLO11 detection output shape");
  }

  const cv::Mat predictions = output.isContinuous() ? output : output.clone();
  const float * values = predictions.ptr<float>();
  const auto value_at = [values, rows, dimensions, channels_first](const int row, const int column) {
      return channels_first ? values[column * rows + row] : values[row * dimensions + column];
    };
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_indices;

  for (int row = 0; row < rows; ++row) {
    int best_class = 0;
    float confidence = value_at(row, 4);
    for (int class_index = 1; class_index < class_count; ++class_index) {
      const float class_confidence = value_at(row, 4 + class_index);
      if (class_confidence > confidence) {
        best_class = class_index;
        confidence = class_confidence;
      }
    }

    if (confidence < confidence_threshold) {
      continue;
    }

    const double center_x = (value_at(row, 0) - padding_x) / resize_scale;
    const double center_y = (value_at(row, 1) - padding_y) / resize_scale;
    const double width = value_at(row, 2) / resize_scale;
    const double height = value_at(row, 3) / resize_scale;
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

  std::vector<Detection2D> detections;
  for (int class_index = 0; class_index < class_count; ++class_index) {
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_scores;
    std::vector<int> source_indices;
    for (std::size_t index = 0; index < boxes.size(); ++index) {
      if (class_indices[index] == class_index) {
        class_boxes.push_back(boxes[index]);
        class_scores.push_back(scores[index]);
        source_indices.push_back(static_cast<int>(index));
      }
    }
    if (class_boxes.empty()) {
      continue;
    }
    std::vector<int> selected;
    cv::dnn::NMSBoxes(
      class_boxes, class_scores, confidence_threshold, nms_threshold, selected);
    for (const int selected_index : selected) {
      const int source_index = source_indices[selected_index];
      detections.push_back(
        {boxes[source_index], class_indices[source_index], scores[source_index]});
    }
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
