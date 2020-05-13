#include "pure_pursuit.h"
#include <iostream>

PurePursuit::PurePursuit(/* args */)
{
}

PurePursuit::~PurePursuit()
{
}

Twist_t PurePursuit::track(const Pose& current_pose, const Pose& target_pose)
{
  double look_ahead = sqrt(pow((target_pose.position.x - current_pose.position.x), 2) +
                           pow((target_pose.position.y - current_pose.position.y), 2));

  double gamma = (2 * (target_pose.position.x - current_pose.position.x)) / pow(look_ahead, 2);

  double vX = 0;

  double vY = look_ahead * LINEAR_KP;

  double vZ = vY * gamma;

  if ((vY * vZ) / 9.81 > 6)
  {
    vY = MIN_LINEAR_VELOCITY;
    vZ = 1.0;
  }

  return Twist_t{ vX, vY, vZ };
}
