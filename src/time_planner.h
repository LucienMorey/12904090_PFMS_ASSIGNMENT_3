#ifndef TIMEPLANNER_H
#define TIMEPLANNER_H

#include "planner.h"
#include "simulator.h"
class TimePlanner : public Planner
{
private:
  /* data */
  Simulator* sim_;

public:
  TimePlanner(std::shared_ptr<Simulator> sim /* args */);
  ~TimePlanner();
  void plan(std::vector<Aircraft> aircraft);
};

#endif
