#ifndef PLANNER_H
#define PLANNER_H

#include "types.h"

class Planner
{
protected:
  std::mutex path_mx_;
  std::vector<Pose> path_;

  GlobalOrd airspace_centre_;
  double airspace_width_;
  double airspace_height_;

  const unsigned int FRIENDLY_KEY = 0;

  const double AVERAGE_LINEAR_VELOCITY = 900.0;

  const GlobalOrd ORIGIN = { 0.0, 0.0 };
  const double DEFAULT_AIRSPACE_LENGTH = 8000.0;
  const double AIRSPACE_EXCLUSION_LENGTH = 200.0;

  /**
   * @brief check if point lies within the airspace range. the test is a point in rectangle test
   *
   * @param point_to_test GlobalOrd bogie position to be tested
   * @return true - if the point intercepts the airspace
   * @return false -if the point does not intercept the airspace
   */
  bool pointInsideSpace(GlobalOrd point_to_test);

public:
  /**
   * @brief Construct a new Planner object. the lanner class is a pure virtual class and will be called with
   * any planner sub class
   *
   */
  Planner();

  /**
   * @brief Destroy the Planner object
   *
   */
  ~Planner(){};

  /**
   * @brief Thread set getter for path containing poses of the friendly then most to least efficient point to track
   *
   * @return std::vector<Pose> - containing current location then poses to track in decending order of efficiency
   */
  std::vector<Pose> getPath();

  /**
   * @brief virtual plan method to be implemented in derived classes
   *
   * @param aircraft vector of aircraft to consider when creating path
   */
  virtual void plan(std::vector<Aircraft> aircraft) = 0;
};

#endif