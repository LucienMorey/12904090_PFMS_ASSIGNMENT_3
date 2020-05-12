#include "pure_pursuit.h"

PurePursuit::PurePursuit(/* args */)
{
}

PurePursuit::~PurePursuit()
{
}

Twist_t PurePursuit::pursue()
{
}

void PurePursuit::setTargetPose(const Pose& target_pose)
{
  target_pose_ = target_pose;
}