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

  double vX, vY, vZ = 0;

  double heading_to_target =
      atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x);

  double current_orientation = atan2(sin(current_pose.orientation), cos(current_pose.orientation));

  heading_to_target = atan2(sin(heading_to_target - current_orientation), cos(heading_to_target - current_orientation));

  // std::cout << "friendly heading " << current_pose.orientation << " target heading " << target_pose.orientation
  //           << " heading to target " << heading_to_target << std::endl;

  if (abs(heading_to_target) > ANGULAR_TOLERANCE)
  {
    vY = MIN_LINEAR_VELOCITY;
    vZ = heading_to_target * ANGULAR_KP;
    vZ = std::min(vZ, (6 * 9.81) / vY);
    vZ = std::max(vZ, -(6 * 9.81 / vY));
    // if (heading_to_target < 0)
    // {
    //   vZ = -(6 * 9.81) / vY;
    // }
    // else
    // {
    //   vZ = (6 * 9.81) / vY;
    // }
  }
  else
  {
    vY = 900.0;
    vZ = 0;

    // vZ = vY * gamma;

    vY = std::max(vY, MIN_LINEAR_VELOCITY);
    vY = std::min(vY, MAX_LINEAR_VELOCITY);
    vY = std::min(vY, fabs((6.0 * 9.81) / vZ));
  }

  std::cout << "vy " << vY << " vz " << vZ << std::endl;

  return Twist_t{ vX, vY, vZ };
}
