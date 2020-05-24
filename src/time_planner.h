#ifndef TIMEPLANNER_H
#define TIMEPLANNER_H

#include "planner.h"
#include "simulator.h"
class TimePlanner : public Planner
{
private:
  double distance_max_look_ahead_ = 4000.0;
  double time_max_look_ahead_ = 3.0;
  double distance_min_look_ahead_ = 700.0;
  double time_min_look_ahead_ = 0.2;

  double time_gradient_ =
      (time_max_look_ahead_ - time_min_look_ahead_) / (distance_max_look_ahead_ - distance_min_look_ahead_);
  double time_y_intercept = -time_gradient_ * distance_min_look_ahead_ + time_min_look_ahead_;

  std::map<int, double> look_ahead_times;

  double path_time_;

public:
  TimePlanner();
  ~TimePlanner();
  void plan(std::vector<Aircraft> aircraft);
  double getPathTime();
};

#endif
