// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#ifndef TURTLEBOT3_EMBODIED_NAVIGATION__PERCEPTION_UTILS_HPP_
#define TURTLEBOT3_EMBODIED_NAVIGATION__PERCEPTION_UTILS_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <string>
#include <vector>

namespace turtlebot3_embodied_navigation
{

struct Detection2D
{
  cv::Rect box;
  int class_index;
  float confidence;
};

struct Point3D
{
  double x;
  double y;
  double z;
};

std::vector<Detection2D> decode_yolox(
  const cv::Mat & output, double resize_scale, const cv::Size & image_size,
  int input_size, int class_count, float confidence_threshold, float nms_threshold);

float median_depth(
  const cv::Mat & depth, const cv::Rect & box, float minimum_depth, float maximum_depth);

Point3D project_pixel(
  double pixel_x, double pixel_y, double depth, double fx, double fy, double cx, double cy);

std::string classify_color(const cv::Mat & bgr_image, const cv::Rect & box);

}  // namespace turtlebot3_embodied_navigation

#endif  // TURTLEBOT3_EMBODIED_NAVIGATION__PERCEPTION_UTILS_HPP_
