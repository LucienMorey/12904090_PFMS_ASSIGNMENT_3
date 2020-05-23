#ifndef PLANNER_H
#define PLANNER_H

#include "types.h"
#include "graph.h"

class Planner : public Graph
{
  struct keyedAircraft
  {
    int key;
    Aircraft aircraft;
  };

protected:
  /* data */
  std::map<int, Aircraft> planes_;

  std::mutex path_mx_;
  std::vector<Pose> path_;

  const double TIME_PREDICTION_CONSTANT = 1;
  const unsigned int FRIENDLY_KEY = 0;

  const double AVERAGE_LINEAR_VELOCITY = 400.0;
  const double AVERAGE_ANGULAR_VELOCITY = 1.17;

public:
  Planner(/* args */){};
  ~Planner(){};

  std::vector<Pose> getPath();
  virtual void plan(std::vector<Aircraft> aircraft) = 0;
};

#endif