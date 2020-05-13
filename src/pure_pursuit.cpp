#include "pure_pursuit.h"

PurePursuit::PurePursuit(/* args */)
{
}

PurePursuit::~PurePursuit()
{
}

Twist_t PurePursuit::pursue(const Pose& current_pose, const Pose& target_pose)
{
  double rho = sqrt(pow((target_pose.position.x - current_pose.position.x), 2) +
                    pow((target_pose.position.y - current_pose.position.y), 2));

  // double heading_error = M_PI_2 - atan()
}
