#include "planner.h"

Planner::Planner()
{
  // set the
  airspace_height_ = DEFAULT_AIRSPACE_LENGTH - AIRSPACE_EXCLUSION_LENGTH;
  airspace_width_ = DEFAULT_AIRSPACE_LENGTH - AIRSPACE_EXCLUSION_LENGTH;

  // set the centre to be at the origin
  airspace_centre_ = ORIGIN;
}

std::vector<Pose> Planner::getPath()
{
  // lock path so that it can't be overwritten
  std::lock_guard<std::mutex> lock(path_mx_);
  return path_;
}

bool Planner::pointInsideSpace(GlobalOrd point_to_test)
{
  // std::cout << airspace_width_ << " " << airspace_height_ << std::endl;
  return ((point_to_test.x >= airspace_centre_.x - (airspace_width_ / 2.0)) &&
          (point_to_test.x <= airspace_centre_.x + (airspace_width_ / 2.0)) &&
          (point_to_test.y <= airspace_centre_.y + (airspace_height_ / 2.0)) &&
          (point_to_test.y >= airspace_centre_.y - (airspace_height_ / 2.0)));
}