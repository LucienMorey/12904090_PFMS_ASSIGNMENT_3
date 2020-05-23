#include "time_planner.h"
#include <iostream>

TimePlanner::TimePlanner(std::shared_ptr<Simulator> sim)
{
  sim_ = sim.get();
}

TimePlanner::~TimePlanner()
{
}

void TimePlanner::plan(std::vector<Aircraft> aircraft)
{
  weightedGraph_.clear();
  planes_.clear();
  int key = 0;
  std::vector<Pose> poses;
  for (auto plane : aircraft)
  {
    addVertex(key);
    planes_.insert(std::make_pair(key, plane));
    key++;
  }

  for (auto planes_key = 1; planes_key != planes_.size(); planes_key++)
  {
    double distance = sqrt(pow(planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(planes_key).pose.position.x, 2) +
                           pow(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(planes_key).pose.position.y, 2));
    double look_ahead_time = time_gradient_ * distance + time_y_intercept;
    look_ahead_time = fmin(look_ahead_time, time_max_look_ahead_);
    look_ahead_time = fmax(look_ahead_time, time_min_look_ahead_);

    std::cout << look_ahead_time << std::endl;

    GlobalOrd point_next_time_step = {
      planes_.at(planes_key).pose.position.x + planes_.at(planes_key).linear_velocity *
                                                   cos(planes_.at(planes_key).pose.orientation) *
                                                   (look_ahead_time + planes_.at(planes_key).timer.elapsed() / 1000.0),
      planes_.at(planes_key).pose.position.y + planes_.at(planes_key).linear_velocity *
                                                   sin(planes_.at(planes_key).pose.orientation) *
                                                   (look_ahead_time + planes_.at(planes_key).timer.elapsed() / 1000.0)
    };

    Pose pose_next_time_step = { point_next_time_step, planes_.at(planes_key).pose.orientation };

    planes_.at(planes_key).currentGoalPose = pose_next_time_step;

    double heading_to_bogie =
        fmod(atan2(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(planes_key).currentGoalPose.position.y,
                   planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(planes_key).currentGoalPose.position.x) +
                 2 * M_PI,
             2 * M_PI);

    double angle_to_bogie = fabs(angle_to_bogie - planes_.at(FRIENDLY_KEY).pose.orientation);
    if (angle_to_bogie > M_PI)
    {
      angle_to_bogie = 2 * M_PI - angle_to_bogie;
    }

    double time_to_bogie =
        sqrt(pow(planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(planes_key).currentGoalPose.position.x, 2) +
             pow(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(planes_key).currentGoalPose.position.y, 2)) /
            AVERAGE_LINEAR_VELOCITY +
        angle_to_bogie / AVERAGE_ANGULAR_VELOCITY;

    double weight = time_to_bogie;
    addEdge(FRIENDLY_KEY, planes_key, weight);
  }

  unsigned int index_to_most_efficient_bogie = 1;
  for (int i = 1; i != weightedGraph_.at(FRIENDLY_KEY).size(); i++)
  {
    if (weightedGraph_.at(FRIENDLY_KEY).at(i) < weightedGraph_.at(FRIENDLY_KEY).at(index_to_most_efficient_bogie))
    {
      index_to_most_efficient_bogie = i;
    }
  }

  std::lock_guard<std::mutex> lock(path_mx_);
  path_.clear();
  path_.push_back(planes_.at(FRIENDLY_KEY).pose);
  path_.push_back(planes_.at(index_to_most_efficient_bogie).currentGoalPose);
  sim_->testPose(path_);
}