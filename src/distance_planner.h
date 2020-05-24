#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include "planner.h"
#include "simulator.h"
#include <iostream>

class DistancePlanner : public Planner
{
public:
  DistancePlanner();
  ~DistancePlanner();
  void plan(std::vector<Aircraft> aircraft);
};

#endif