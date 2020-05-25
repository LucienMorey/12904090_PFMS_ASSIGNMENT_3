#ifndef PLANNER_H
#define PLANNER_H

#include "types.h"

class Planner
{
protected:
  std::mutex path_mx_;
  std::vector<Pose> path_;

  const unsigned int FRIENDLY_KEY = 0;

  const double AVERAGE_LINEAR_VELOCITY = 900.0;

  bool pointInsideSpace(GlobalOrd point_to_test);

  GlobalOrd airspace_centre_;
  double airspace_width_;
  double airspace_height_;

  const GlobalOrd ORIGIN = { 0.0, 0.0 };
  const double DEFAULT_AIRSPACE_LENGTH = 8000.0;
  const double AIRSPACE_EXCLUSION_LENGTH = 200.0;

public:
  Planner();
  ~Planner(){};

  std::vector<Pose> getPath();
  virtual void plan(std::vector<Aircraft> aircraft) = 0;
};

#endif