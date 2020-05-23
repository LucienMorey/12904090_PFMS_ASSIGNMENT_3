#include "planner.h"

std::vector<Pose> Planner::getPath()
{
  std::lock_guard<std::mutex> lock(path_mx_);
  return path_;
}