#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include "types.h"

struct Twist_t
{
  double vX;
  double vY;
  double vZ;
};

class PathTracker
{
protected:
  const double ANGULAR_TOLERANCE = M_PI / 80;

  const double MIN_LINEAR_VELOCITY = 50.0;
  const double MAX_LINEAR_VELOCITY = 900.0;
  const double MAX_G = 6.0;
  const double G = 9.81;
  const double MAX_ANGLE_VELOCITY = G * MAX_G / MIN_LINEAR_VELOCITY;

public:
  PathTracker(){};

  virtual Twist_t track(const Pose& current_pose, double current_velocity, const Pose& initial_pose,
                        const Pose& target_pose) = 0;
};

#endif
