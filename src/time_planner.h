#ifndef TIMEPLANNER_H
#define TIMEPLANNER_H

#include "planner.h"
#include "simulator.h"
class TimePlanner : public Planner
{
public:
  TimePlanner();
  ~TimePlanner();
  void plan(std::vector<Aircraft> aircraft);
  double getPathTime();
};

#endif
