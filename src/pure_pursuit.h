#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "path_tracker.h"

class PurePursuit : public path_tracker
{
private:
  /* data */

  double LINEAR_KP = 0.9;
  double ANGULAR_KP = 0.8;

public:
  PurePursuit();
  ~PurePursuit();
  Twist_t track(const Pose& current_pose, const Pose& target_pose);
};

#endif