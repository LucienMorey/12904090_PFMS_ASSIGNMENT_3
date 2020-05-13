#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include "types.h"

struct Twist_t
{
  double vX;
  double vY;
  double vZ;
};

class path_tracker
{
protected:
  const double LINEAR_TOLERANCE = 200.0;
  const double ANGULAR_TOLERANCE = M_PI / 36;

  const unsigned int MAX_G = 6;

  const double MAX_LINEAR_VELOCITY = 900.0;
  const double MIN_LINEAR_VELOCITY = 50.0;

public:
  path_tracker(){};

  virtual Twist_t track(const Pose& current_pose, const Pose& target_pose) = 0;
};

#endif
