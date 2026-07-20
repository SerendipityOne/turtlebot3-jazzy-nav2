// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#ifndef TURTLEBOT3_EMBODIED_NAVIGATION__DETECTION_HISTORY_HPP_
#define TURTLEBOT3_EMBODIED_NAVIGATION__DETECTION_HISTORY_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>

namespace turtlebot3_embodied_navigation
{

// Tracks whether one spatially-associated target was observed in a sliding frame window.
class DetectionHistory
{
public:
  DetectionHistory(std::size_t window_size, std::size_t required_matches);

  bool add(bool is_match);
  bool confirmed() const;
  void reset();

  std::size_t size() const;
  std::size_t match_count() const;

private:
  std::size_t window_size_;
  std::size_t required_matches_;
  std::size_t match_count_{0};
  std::deque<uint8_t> matches_;
};

}  // namespace turtlebot3_embodied_navigation

#endif  // TURTLEBOT3_EMBODIED_NAVIGATION__DETECTION_HISTORY_HPP_
