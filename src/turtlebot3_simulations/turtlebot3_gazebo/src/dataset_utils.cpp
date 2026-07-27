#include "turtlebot3_gazebo/dataset_utils.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace turtlebot3_gazebo
{

namespace
{

std::vector<std::string> split_line(const std::string & line, const char delimiter)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, delimiter)) {
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

std::vector<DatasetAsset> load_asset_manifest(const std::filesystem::path & manifest_path)
{
  std::ifstream stream(manifest_path);
  if (!stream) {
    throw std::runtime_error("cannot open asset manifest: " + manifest_path.string());
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("asset manifest is empty");
  }
  const auto header = split_line(line, '\t');
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size(); ++index) {
    columns.emplace(header[index], index);
  }
  const std::array<const char *, 11> required = {
    "asset_id", "target_class", "class_id", "color", "surface", "split", "sdf_path",
    "source_url", "license", "license_url", "sha256"};
  for (const auto * name : required) {
    if (!columns.count(name)) {
      throw std::runtime_error("asset manifest missing column: " + std::string(name));
    }
  }

  std::vector<DatasetAsset> assets;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = split_line(line, '\t');
    const auto value = [&](const std::string & name) -> const std::string & {
        const auto index = columns.at(name);
        if (index >= fields.size()) {
          throw std::runtime_error("malformed asset manifest row");
        }
        return fields[index];
      };
    assets.push_back(DatasetAsset{
      value("asset_id"), value("target_class"), std::stoi(value("class_id")), value("color"),
      value("surface"), columns.count("z_offset") ? std::stod(value("z_offset")) : 0.0,
      value("split"), manifest_path.parent_path() / value("sdf_path"),
      value("source_url"), value("license"), value("license_url"), value("sha256")});
  }
  return assets;
}

AcceptanceDecision evaluate_acceptance_case(
  const std::vector<AcceptanceFrame> & frames, const std::string & expected_class,
  const std::vector<std::string> & enabled_classes, const std::size_t required_matches,
  const bool is_negative)
{
  if (frames.empty() || required_matches == 0 || required_matches > frames.size()) {
    throw std::invalid_argument("invalid acceptance frame window");
  }
  const std::set<std::string> enabled(enabled_classes.begin(), enabled_classes.end());
  if (!is_negative && !enabled.count(expected_class)) {
    throw std::invalid_argument("expected class is not enabled");
  }

  std::map<std::string, std::size_t> class_frames;
  std::size_t expected_location_matches = 0;
  std::size_t bottle_frames = 0;
  for (const auto & frame : frames) {
    const std::set<std::string> unique_classes(
      frame.detected_classes.begin(), frame.detected_classes.end());
    bottle_frames += unique_classes.count("bottle");
    for (const auto & name : enabled) {
      class_frames[name] += unique_classes.count(name);
    }
    expected_location_matches += frame.has_valid_expected_location;
  }

  std::vector<std::string> confirmed_classes;
  for (const auto & [name, count] : class_frames) {
    if (count >= required_matches) {
      confirmed_classes.push_back(name);
    }
  }
  const bool class_gate = is_negative ? confirmed_classes.empty() :
    confirmed_classes == std::vector<std::string>{expected_class};
  const bool location_gate = is_negative || expected_location_matches >= required_matches;
  return AcceptanceDecision{
    class_gate && location_gate && bottle_frames == 0,
    expected_location_matches, bottle_frames, confirmed_classes};
}

CapturePose make_hard_negative_pose(
  const std::size_t sample_index, const std::uint64_t seed,
  const std::vector<std::array<double, 2>> & base_positions, const double maximum_jitter)
{
  if (base_positions.empty() || maximum_jitter < 0.0) {
    throw std::invalid_argument("hard-negative capture positions are invalid");
  }
  std::mt19937_64 randomizer(seed * 1000003ULL + sample_index);
  std::uniform_real_distribution<double> jitter(-maximum_jitter, maximum_jitter);
  std::uniform_real_distribution<double> yaw(-3.141592653589793, 3.141592653589793);
  const auto & base = base_positions.at(sample_index % base_positions.size());
  return CapturePose{base[0] + jitter(randomizer), base[1] + jitter(randomizer), yaw(randomizer)};
}

std::vector<InstanceBox> decode_instance_boxes(
  const std::vector<std::uint8_t> & data, const std::uint32_t width,
  const std::uint32_t height, const std::uint32_t step,
  const std::size_t minimum_visible_pixels, const int minimum_box_dimension,
  const std::uint8_t maximum_class_label)
{
  if (width == 0 || height == 0 || step < width * 3 || data.size() < step * height) {
    throw std::invalid_argument("invalid RGB8 instance label map");
  }
  if (minimum_visible_pixels == 0 || minimum_box_dimension <= 0 || maximum_class_label == 0) {
    throw std::invalid_argument("invalid instance box filter");
  }

  std::unordered_map<std::uint32_t, InstanceBox> boxes;
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const std::size_t offset = static_cast<std::size_t>(y) * step + x * 3;
      // Gazebo stores the 8-bit semantic label in B and the 16-bit instance count in R/G.
      const auto label = data[offset + 2];
      if (label == 0 || label > maximum_class_label) {
        continue;
      }
      const auto instance = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(data[offset + 1]) * 256U + data[offset]);
      const std::uint32_t key = static_cast<std::uint32_t>(label) << 16U | instance;
      const auto [iterator, inserted] = boxes.try_emplace(
        key, InstanceBox{label, instance, 0, static_cast<int>(x), static_cast<int>(y),
          static_cast<int>(x), static_cast<int>(y)});
      auto & box = iterator->second;
      if (!inserted) {
        box.min_x = std::min(box.min_x, static_cast<int>(x));
        box.min_y = std::min(box.min_y, static_cast<int>(y));
        box.max_x = std::max(box.max_x, static_cast<int>(x));
        box.max_y = std::max(box.max_y, static_cast<int>(y));
      }
      ++box.visible_pixels;
    }
  }

  std::vector<InstanceBox> result;
  result.reserve(boxes.size());
  for (const auto & [key, box] : boxes) {
    (void)key;
    const int box_width = box.max_x - box.min_x + 1;
    const int box_height = box.max_y - box.min_y + 1;
    if (box.visible_pixels >= minimum_visible_pixels &&
      box_width >= minimum_box_dimension && box_height >= minimum_box_dimension)
    {
      result.push_back(box);
    }
  }
  std::sort(result.begin(), result.end(), [](const auto & left, const auto & right) {
      return std::tie(left.class_id, left.instance_id) <
             std::tie(right.class_id, right.instance_id);
  });
  return result;
}

bool is_box_oversized(
  const InstanceBox & box, const std::uint32_t width, const std::uint32_t height,
  const double maximum_area_ratio)
{
  if (width == 0 || height == 0 || maximum_area_ratio <= 0.0 || maximum_area_ratio > 1.0 ||
    box.min_x < 0 || box.min_y < 0 || box.max_x < box.min_x || box.max_y < box.min_y ||
    box.max_x >= static_cast<int>(width) || box.max_y >= static_cast<int>(height))
  {
    throw std::invalid_argument("invalid maximum box area filter");
  }
  const double box_area =
    static_cast<double>(box.max_x - box.min_x + 1) * (box.max_y - box.min_y + 1);
  return box_area / (static_cast<double>(width) * height) > maximum_area_ratio;
}

double intersection_over_union(const PixelBox & left, const PixelBox & right)
{
  if (left.width <= 0.0 || left.height <= 0.0 || right.width <= 0.0 || right.height <= 0.0) {
    throw std::invalid_argument("IoU requires positive box dimensions");
  }
  const double intersection_width = std::max(
    0.0, std::min(left.x + left.width, right.x + right.width) - std::max(left.x, right.x));
  const double intersection_height = std::max(
    0.0, std::min(left.y + left.height, right.y + right.height) - std::max(left.y, right.y));
  const double intersection = intersection_width * intersection_height;
  return intersection / (left.width * left.height + right.width * right.height - intersection);
}

std::size_t round_robin_index(const std::size_t ordinal, const std::size_t candidate_count)
{
  if (candidate_count == 0) {
    throw std::invalid_argument("round-robin selection requires at least one candidate");
  }
  return ordinal % candidate_count;
}

std::string to_yolo_line(
  const InstanceBox & box, const std::uint32_t width, const std::uint32_t height)
{
  if (width == 0 || height == 0 || box.class_id == 0 || box.min_x < 0 || box.min_y < 0 ||
    box.max_x < box.min_x || box.max_y < box.min_y ||
    box.max_x >= static_cast<int>(width) || box.max_y >= static_cast<int>(height))
  {
    throw std::invalid_argument("box is outside the image");
  }
  const double box_width = box.max_x - box.min_x + 1;
  const double box_height = box.max_y - box.min_y + 1;
  const double center_x = box.min_x + box_width / 2.0;
  const double center_y = box.min_y + box_height / 2.0;

  std::ostringstream line;
  line << static_cast<int>(box.class_id - 1) << std::fixed << std::setprecision(8) << ' '
       << center_x / width << ' ' << center_y / height << ' '
       << box_width / width << ' ' << box_height / height;
  return line.str();
}

}  // namespace turtlebot3_gazebo
