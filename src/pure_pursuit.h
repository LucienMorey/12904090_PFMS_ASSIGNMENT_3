#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "types.h"

struct Twist_t
{
  double vY;
  double vX;
  double vZ;
};

class PurePursuit
{
private:
  /* data */

  unsigned int max_g_;

  double max_linear_velocity_;
  double min_linear_velocity_;

  double LINEAR_KP = 1.0;
  double ANGULAR_KD = 1.0;

  const double LINEAR_TOLERANCE = 200.0;

public:
  PurePursuit(/* args */);
  ~PurePursuit();
  Twist_t pursue(const Pose& current_pose, const Pose& target_pose);
};

#endif