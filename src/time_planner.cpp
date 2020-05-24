#include "time_planner.h"
#include <iostream>

TimePlanner::TimePlanner()
{
}

TimePlanner::~TimePlanner()
{
}

void TimePlanner::plan(std::vector<Aircraft> aircraft)
{
  weightedGraph_.clear();
  planes_.clear();

  // create weighted graph with reference to the aircraft
  int key = 0;
  for (auto plane : aircraft)
  {
    addVertex(key);
    planes_.insert(std::make_pair(key, plane));
    key++;
  }

  // loop through aricraft and determine the difficulty to track
  for (auto planes_key = 1; planes_key != planes_.size(); planes_key++)
  {
    // calculate the distance to the bogie
    double distance = sqrt(pow(planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(planes_key).pose.position.x, 2) +
                           pow(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(planes_key).pose.position.y, 2));

    // the appropriate lookahead time can be calculated based on an appropriate line function
    double look_ahead_time = time_gradient_ * distance + time_y_intercept;

    // cap out lookahead time
    look_ahead_time = fmin(look_ahead_time, time_max_look_ahead_);
    look_ahead_time = fmax(look_ahead_time, time_min_look_ahead_);

    // record bogie lookahead time
    look_ahead_times[planes_key] = look_ahead_time;

    // calculate bogie position in at the end of the look ahead time
    GlobalOrd point_next_time_step = {
      planes_.at(planes_key).pose.position.x + planes_.at(planes_key).linear_velocity *
                                                   cos(planes_.at(planes_key).pose.orientation) *
                                                   (look_ahead_time + planes_.at(planes_key).timer.elapsed() / 1000.0),
      planes_.at(planes_key).pose.position.y + planes_.at(planes_key).linear_velocity *
                                                   sin(planes_.at(planes_key).pose.orientation) *
                                                   (look_ahead_time + planes_.at(planes_key).timer.elapsed() / 1000.0)
    };

    // assume the bogie stays at the same orientation and create pose based on forcast position
    Pose pose_next_time_step = { point_next_time_step, planes_.at(planes_key).pose.orientation };

    // the bogie goal pose is the pose in the next time step
    planes_.at(planes_key).currentGoalPose = pose_next_time_step;

    // heading between friendly and bogie
    double heading_to_bogie =
        fmod(atan2(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(planes_key).currentGoalPose.position.y,
                   planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(planes_key).currentGoalPose.position.x) +
                 2 * M_PI,
             2 * M_PI);

    // the angle to bogie is the difference between the heading to the bogie and the current orientaiton
    double angle_to_bogie = fabs(angle_to_bogie - planes_.at(FRIENDLY_KEY).pose.orientation);

    // constrain angle so that error is never bigger than pi
    if (angle_to_bogie > M_PI)
    {
      angle_to_bogie = 2 * M_PI - angle_to_bogie;
    }

    // estimate time to bogie
    // currently only done based on linear distance and not heading error
    double time_to_bogie =
        sqrt(pow(planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(planes_key).currentGoalPose.position.x, 2) +
             pow(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(planes_key).currentGoalPose.position.y, 2)) /
        AVERAGE_LINEAR_VELOCITY;

    //+angle_to_bogie / AVERAGE_ANGULAR_VELOCITY;

    // weight graph with time to bogie
    double weight = time_to_bogie;
    addEdge(FRIENDLY_KEY, planes_key, weight);
  }

  // determine the bogie wih the shortest estimated time to convergence
  unsigned int index_to_most_efficient_bogie = 1;
  for (int i = 1; i != weightedGraph_.at(FRIENDLY_KEY).size(); i++)
  {
    if (weightedGraph_.at(FRIENDLY_KEY).at(i) < weightedGraph_.at(FRIENDLY_KEY).at(index_to_most_efficient_bogie))
    {
      index_to_most_efficient_bogie = i;
    }
  }

  // lock and create new path
  std::lock_guard<std::mutex> lock(path_mx_);
  path_.clear();
  path_.push_back(planes_.at(FRIENDLY_KEY).pose);
  path_.push_back(planes_.at(index_to_most_efficient_bogie).currentGoalPose);
  path_time_ = look_ahead_times.at(index_to_most_efficient_bogie);
}

double TimePlanner::getPathTime()
{
  std::lock_guard<std::mutex> lock(path_mx_);
  return path_time_;
}