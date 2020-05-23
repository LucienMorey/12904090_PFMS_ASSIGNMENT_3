#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include "types.h"
#include "graph.h"
#include "simulator.h"
#include <iostream>

class DistancePlanner : public Graph
{
  struct keyedAircraft
  {
    int key;
    Aircraft aircraft;
  };

private:
  /* data */
  std::map<int, Aircraft> planes_;

  std::mutex path_mx_;
  std::vector<Pose> path_;

  const double TIME_PREDICTION_CONSTANT = 1;
  const unsigned int FRIENDLY_KEY = 0;

  Simulator* sim_;

public:
  DistancePlanner(std::shared_ptr<Simulator> sim);
  ~DistancePlanner();
  std::vector<Pose> getPath();
  void plan(std::vector<Aircraft> aircraft);
};

#endif