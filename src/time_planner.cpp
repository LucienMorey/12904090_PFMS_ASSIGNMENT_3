#include "time_planner.h"

TimePlanner::TimePlanner()
{
}

TimePlanner::~TimePlanner()
{
}

void TimePlanner::plan(std::vector<Aircraft> aircraft)
{
  std::map<double, int> weighted_map;
  std::map<int, Aircraft> planes;

  // create weighted graph with reference to the aircraft
  int key = 0;
  for (auto plane : aircraft)
  {
    planes.insert(std::make_pair(key, plane));
    key++;
  }

  // loop through aricraft and determine the difficulty to track
  for (auto planes_key = 1; planes_key != planes.size(); planes_key++)
  {
    // calculate the distance to the bogie
    double distance = sqrt(pow(planes.at(FRIENDLY_KEY).pose.position.x - planes.at(planes_key).pose.position.x, 2) +
                           pow(planes.at(FRIENDLY_KEY).pose.position.y - planes.at(planes_key).pose.position.y, 2));

    double look_ahead_time = distance / AVERAGE_LINEAR_VELOCITY;
    // calculate bogie position in at the end of the look ahead time
    GlobalOrd point_next_time_step = {
      planes.at(planes_key).pose.position.x + planes.at(planes_key).linear_velocity *
                                                  cos(planes.at(planes_key).pose.orientation) *
                                                  (look_ahead_time + planes.at(planes_key).timer.elapsed() / 1000.0),
      planes.at(planes_key).pose.position.y + planes.at(planes_key).linear_velocity *
                                                  sin(planes.at(planes_key).pose.orientation) *
                                                  (look_ahead_time + planes.at(planes_key).timer.elapsed() / 1000.0)
    };

    // assume the bogie stays at the same orientation and create pose based on forcast position
    Pose pose_next_time_step = { point_next_time_step, planes.at(planes_key).pose.orientation };

    // the bogie goal pose is the pose in the next time step
    planes.at(planes_key).currentGoalPose = pose_next_time_step;

    // estimate time to bogie
    double time_to_bogie =
        sqrt(pow(planes.at(FRIENDLY_KEY).pose.position.x - planes.at(planes_key).currentGoalPose.position.x, 2) +
             pow(planes.at(FRIENDLY_KEY).pose.position.y - planes.at(planes_key).currentGoalPose.position.y, 2)) /
        AVERAGE_LINEAR_VELOCITY;

    // weight graph with time to bogie
    double weight = time_to_bogie;
    // rely on map autosorting lowest weight to front
    weighted_map[weight] = planes_key;
  }

  // lock and create new path
  std::lock_guard<std::mutex> lock(path_mx_);
  path_.clear();
  path_.push_back(planes.at(FRIENDLY_KEY).pose);
  for (auto key : weighted_map)
  {
    path_.push_back(planes.at(key.second).currentGoalPose);
  }
}