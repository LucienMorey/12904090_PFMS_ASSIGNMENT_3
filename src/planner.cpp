#include "planner.h"
/**
 * @brief Construct a new Planner:: Planner object. the lanner class is a pure virtual class and will be called with any
 * planner sub class
 *
 */
Planner::Planner()
{
  // set the airspace dimensions with a margin of error
  airspace_height_ = DEFAULT_AIRSPACE_LENGTH - AIRSPACE_EXCLUSION_LENGTH;
  airspace_width_ = DEFAULT_AIRSPACE_LENGTH - AIRSPACE_EXCLUSION_LENGTH;

  // set the centre to be at the origin
  airspace_centre_ = ORIGIN;
}

/**
 * @brief Thread set getter for path containing poses of the friendly then most to least efficient point to track
 *
 * @return std::vector<Pose> - containing current location then poses to track in decending order of efficiency
 */
std::vector<Pose> Planner::getPath()
{
  // lock path so that it can't be overwritten
  std::lock_guard<std::mutex> lock(path_mx_);
  return path_;
}

/**
 * @brief check if point lies within the airspace range. the test is a point in rectangle test
 *
 * @param point_to_test GlobalOrd bogie position to be tested
 * @return true - if the point intercepts the airspace
 * @return false -if the point does not intercept the airspace
 */
bool Planner::pointInsideSpace(GlobalOrd point_to_test)
{
  return ((point_to_test.x >= airspace_centre_.x - (airspace_width_ / 2.0)) &&
          (point_to_test.x <= airspace_centre_.x + (airspace_width_ / 2.0)) &&
          (point_to_test.y <= airspace_centre_.y + (airspace_height_ / 2.0)) &&
          (point_to_test.y >= airspace_centre_.y - (airspace_height_ / 2.0)));
}