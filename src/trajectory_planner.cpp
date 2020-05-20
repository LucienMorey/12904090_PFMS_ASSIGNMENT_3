#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner(/* args */)
{
}

TrajectoryPlanner::~TrajectoryPlanner()
{
}

void TrajectoryPlanner::plan(std::vector<Aircraft> Aircraft)
{
  weightedGraph_.clear();
  int key = 0;
  for (auto plane : Aircraft)
  {
    addVertex(key);
    planes_.insert(std::make_pair(key, plane));
    key++;
  }

  for (auto planes_key = 1; planes_key != planes_.size() - 1; planes_key++)
  {
    GlobalOrd point_next_time_step = { planes_.at(planes_key).pose.position.x +
                                           planes_.at(planes_key).linear_velocity *
                                               cos(planes_.at(planes_key).pose.orientation) * TIME_PREDICTION_CONSTANT,
                                       planes_.at(planes_key).pose.position.y +
                                           planes_.at(planes_key).linear_velocity *
                                               sin(planes_.at(planes_key).pose.orientation) *
                                               TIME_PREDICTION_CONSTANT };

    Pose pose_next_time_step = { point_next_time_step, planes_.at(planes_key).pose.orientation };

    planes_.at(planes_key).pose = pose_next_time_step;

    double distance = sqrt(pow(pose_next_time_step.position.x - planes_.at(FRIENDLY_KEY).pose.position.x, 2) +
                           pow(pose_next_time_step.position.y - planes_.at(FRIENDLY_KEY).pose.position.y, 2));

    double angle_to_friendly = atan2(planes_.at(FRIENDLY_KEY).pose.position.x - pose_next_time_step.position.x,
                                     planes_.at(FRIENDLY_KEY).pose.position.y - pose_next_time_step.position.y);

    double converted_orientation = atan2(sin(pose_next_time_step.orientation), cos(pose_next_time_step.orientation));

    double angle_error_bogie_to_friendly = fabs(angle_to_friendly - converted_orientation);

    if (angle_error_bogie_to_friendly > M_PI)
    {
      angle_error_bogie_to_friendly = 2 * M_PI - angle_error_bogie_to_friendly;
    }

    double weight = pow(distance, angle_error_bogie_to_friendly);
    addEdge(FRIENDLY_KEY, planes_key, weight);
  }
  unsigned int index_to_most_efficient_bogie = 1;
  for (int i = 1; i != weightedGraph_.at(FRIENDLY_KEY).size(); i++)
  {
    if (weightedGraph_.at(FRIENDLY_KEY).at(i) > weightedGraph_.at(FRIENDLY_KEY).at(index_to_most_efficient_bogie))
    {
      index_to_most_efficient_bogie = i;
    }
  }

  std::lock_guard<std::mutex> lock(path_mx_);
  path_.clear();
  path_.push_back(planes_.at(FRIENDLY_KEY).pose);
  path_.push_back(planes_.at(index_to_most_efficient_bogie).pose);
}

std::vector<Pose> TrajectoryPlanner::getPath()
{
  std::lock_guard<std::mutex> lock(path_mx_);
  return path_;
}
