#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "path_tracker.h"

class PurePursuit : public path_tracker
{
private:
  /* data */

  double LINEAR_KP = 1.0;
  double ANGULAR_KD = 1.0;

  unsigned int max_g_;

  double max_linear_velocity_;
  double min_linear_velocity_;

public:
  PurePursuit();
  ~PurePursuit();
  Twist_t pursue(const Pose& current_pose, const Pose& target_pose);
};

#endif