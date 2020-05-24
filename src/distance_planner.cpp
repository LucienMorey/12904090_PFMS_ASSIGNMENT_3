#include "distance_planner.h"

DistancePlanner::DistancePlanner()
{
}

DistancePlanner::~DistancePlanner()
{
}

void DistancePlanner::plan(std::vector<Aircraft> Aircraft)
{
  weightedGraph_.clear();
  planes_.clear();

  int key = 0;
  // create gempy graph of planes and keyed map of aircraft to match
  for (auto plane : Aircraft)
  {
    addVertex(key);
    planes_.insert(std::make_pair(key, plane));
    key++;
  }

  for (auto planes_key = 1; planes_key != planes_.size(); planes_key++)
  {
    // calculate bogie position in the next time step
    GlobalOrd point_next_time_step = {
      planes_.at(planes_key).pose.position.x +
          planes_.at(planes_key).linear_velocity * cos(planes_.at(planes_key).pose.orientation) *
              (TIME_PREDICTION_CONSTANT + planes_.at(planes_key).timer.elapsed() / 1000.0),
      planes_.at(planes_key).pose.position.y +
          planes_.at(planes_key).linear_velocity * sin(planes_.at(planes_key).pose.orientation) *
              (TIME_PREDICTION_CONSTANT + planes_.at(planes_key).timer.elapsed() / 1000.0)
    };

    // bogie pose in next timestep will be at the next position with the same orientation
    Pose pose_next_time_step = { point_next_time_step, planes_.at(planes_key).pose.orientation };

    // set goal pose for next time step
    planes_.at(planes_key).currentGoalPose = pose_next_time_step;

    // calculate the distance between the bogie position and the friendly position
    double distance = sqrt(pow(pose_next_time_step.position.x - planes_.at(FRIENDLY_KEY).pose.position.x, 2) +
                           pow(pose_next_time_step.position.y - planes_.at(FRIENDLY_KEY).pose.position.y, 2));

    // weight graph based on distance to bogie
    double weight = distance;
    addEdge(FRIENDLY_KEY, planes_key, weight);
  }

  // find the index for the lowest weighted graph edge
  unsigned int index_to_most_efficient_bogie = 1;
  for (int i = 1; i != weightedGraph_.at(FRIENDLY_KEY).size(); i++)
  {
    if (weightedGraph_.at(FRIENDLY_KEY).at(i) < weightedGraph_.at(FRIENDLY_KEY).at(index_to_most_efficient_bogie))
    {
      index_to_most_efficient_bogie = i;
    }
  }

  // lock the path mutex and set the path to be the link between the friendly and lowest weighted bogie
  std::lock_guard<std::mutex> lock(path_mx_);
  path_.clear();
  path_.push_back(planes_.at(FRIENDLY_KEY).pose);
  path_.push_back(planes_.at(index_to_most_efficient_bogie).currentGoalPose);
}
