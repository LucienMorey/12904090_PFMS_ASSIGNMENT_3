#ifndef TIMEPLANNER_H
#define TIMEPLANNER_H

#include "planner.h"
#include "simulator.h"
class TimePlanner : public Planner
{
private:
  const double DEFAULT_MAX_LOOK_AHEAD_DISTANCE = 4000.0;
  const double DEFAULT_MAX_LOOK_AHEAD_TIME = 3.0;
  const double DEFAULT_MIN_LOOK_AHEAD_DISTANCE = 700.0;
  const double DEFAULT_MIN_LOOK_AHEAD_TIME = 0.2;

  double distance_max_look_ahead_;
  double time_max_look_ahead_;
  double distance_min_look_ahead_;
  double time_min_look_ahead_;

  double time_gradient_;
  double time_y_intercept;

  std::map<int, double> look_ahead_times;

  double path_time_;

public:
  TimePlanner();
  ~TimePlanner();
  void plan(std::vector<Aircraft> aircraft);
  double getPathTime();
};

#endif
