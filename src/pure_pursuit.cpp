#include "pure_pursuit.h"
#include <iostream>
#include <algorithm>

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

  vY = std::max(vY, MIN_LINEAR_VELOCITY);
  vY = std::min(vY, MAX_LINEAR_VELOCITY);

  if (((vY * vZ) / 9.81) > 6)
  {
    vY = MIN_LINEAR_VELOCITY;
    vZ = target_pose.orientation - current_pose.orientation * 0.001;
  }

  return Twist_t{ vX, vY, vZ };
}
