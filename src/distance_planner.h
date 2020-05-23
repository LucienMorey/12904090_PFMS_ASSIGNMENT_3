#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include "planner.h"
#include "simulator.h"
#include <iostream>

class DistancePlanner : public Planner
{
private:
  /* data */

  Simulator* sim_;

public:
  DistancePlanner(std::shared_ptr<Simulator> sim);
  ~DistancePlanner();
  std::vector<Pose> getPath();
  void plan(std::vector<Aircraft> aircraft);
};

#endif