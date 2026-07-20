// Copyright 2026 Serendipity
// Licensed under the Apache License, Version 2.0

#include "turtlebot3_embodied_navigation/detection_history.hpp"

#include <stdexcept>

namespace turtlebot3_embodied_navigation
{

DetectionHistory::DetectionHistory(
  const std::size_t window_size, const std::size_t required_matches)
: window_size_(window_size), required_matches_(required_matches)
{
  if (window_size_ == 0 || required_matches_ == 0 || required_matches_ > window_size_) {
    throw std::invalid_argument("detection window requires 0 < matches <= window size");
  }
}

bool DetectionHistory::add(const bool is_match)
{
  matches_.push_back(is_match ? 1U : 0U);
  match_count_ += is_match ? 1U : 0U;

  if (matches_.size() > window_size_) {
    match_count_ -= matches_.front();
    matches_.pop_front();
  }

  return confirmed();
}

bool DetectionHistory::confirmed() const
{
  return match_count_ >= required_matches_;
}

void DetectionHistory::reset()
{
  matches_.clear();
  match_count_ = 0;
}

std::size_t DetectionHistory::size() const
{
  return matches_.size();
}

std::size_t DetectionHistory::match_count() const
{
  return match_count_;
}

}  // namespace turtlebot3_embodied_navigation
