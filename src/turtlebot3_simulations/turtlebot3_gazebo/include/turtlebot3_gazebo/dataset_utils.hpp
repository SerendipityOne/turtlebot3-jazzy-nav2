#ifndef TURTLEBOT3_GAZEBO__DATASET_UTILS_HPP_
#define TURTLEBOT3_GAZEBO__DATASET_UTILS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace turtlebot3_gazebo
{

struct InstanceBox
{
  std::uint8_t class_id;
  std::uint16_t instance_id;
  std::size_t visible_pixels;
  int min_x;
  int min_y;
  int max_x;
  int max_y;
};

// Axis-aligned pixel box used to compare detector output with Gazebo segmentation truth.
struct PixelBox
{
  double x;
  double y;
  double width;
  double height;
};

struct DatasetAsset
{
  std::string id;
  std::string target_class;
  int class_id;
  std::string color;
  std::string surface;
  double z_offset;
  std::string split;
  std::filesystem::path sdf_path;
  std::string source_url;
  std::string license;
  std::string license_url;
  std::string sha256;
};

struct AcceptanceFrame
{
  std::vector<std::string> detected_classes;
  bool has_valid_expected_location{false};
};

struct AcceptanceDecision
{
  bool passed;
  std::size_t expected_location_matches;
  std::size_t bottle_frames;
  std::vector<std::string> confirmed_classes;
};

struct CapturePose
{
  double x;
  double y;
  double yaw;
};

std::vector<DatasetAsset> load_asset_manifest(const std::filesystem::path & manifest_path);

AcceptanceDecision evaluate_acceptance_case(
  const std::vector<AcceptanceFrame> & frames, const std::string & expected_class,
  const std::vector<std::string> & enabled_classes, std::size_t required_matches,
  bool is_negative);

CapturePose make_hard_negative_pose(
  std::size_t sample_index, std::uint64_t seed,
  const std::vector<std::array<double, 2>> & base_positions, double maximum_jitter);

std::vector<InstanceBox> decode_instance_boxes(
  const std::vector<std::uint8_t> & data, std::uint32_t width, std::uint32_t height,
  std::uint32_t step, std::size_t minimum_visible_pixels, int minimum_box_dimension,
  std::uint8_t maximum_class_label = 5);

bool is_box_oversized(
  const InstanceBox & box, std::uint32_t width, std::uint32_t height,
  double maximum_area_ratio);

double intersection_over_union(const PixelBox & left, const PixelBox & right);

std::size_t round_robin_index(std::size_t ordinal, std::size_t candidate_count);

std::string to_yolo_line(const InstanceBox & box, std::uint32_t width, std::uint32_t height);

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__DATASET_UTILS_HPP_
