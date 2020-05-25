#include "planner.h"

std::vector<Pose> Planner::getPath()
{
  // lock path so that it can't be overwritten
  std::lock_guard<std::mutex> lock(path_mx_);
  return path_;
}