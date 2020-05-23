#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<Simulator> sim)
{
  sim_ = sim.get();
}

TrajectoryPlanner::~TrajectoryPlanner()
{
}

void TrajectoryPlanner::plan(std::vector<Aircraft> Aircraft)
{
  weightedGraph_.clear();
  planes_.clear();
  int key = 0;
  std::vector<Pose> poses;
  for (auto plane : Aircraft)
  {
    addVertex(key);
    planes_.insert(std::make_pair(key, plane));
    key++;
  }

  std::cout << "friendly at " << planes_.at(FRIENDLY_KEY).pose.position.x << " "
            << planes_.at(FRIENDLY_KEY).pose.position.y << std::endl;

  for (auto planes_key = 1; planes_key != planes_.size(); planes_key++)
  {
    // std::cout << planes_.at(planes_key).timer.elapsed() << std::endl;
    GlobalOrd point_next_time_step = {
      planes_.at(planes_key).pose.position.x +
          planes_.at(planes_key).linear_velocity * cos(planes_.at(planes_key).pose.orientation) *
              (TIME_PREDICTION_CONSTANT + planes_.at(planes_key).timer.elapsed() / 1000.0),
      planes_.at(planes_key).pose.position.y +
          planes_.at(planes_key).linear_velocity * sin(planes_.at(planes_key).pose.orientation) *
              (TIME_PREDICTION_CONSTANT + planes_.at(planes_key).timer.elapsed() / 1000.0)
    };

    Pose pose_next_time_step = { point_next_time_step, planes_.at(planes_key).pose.orientation };
    // poses.push_back(pose_next_time_step);

    planes_.at(planes_key).pose = pose_next_time_step;

    double distance = sqrt(pow(pose_next_time_step.position.x - planes_.at(FRIENDLY_KEY).pose.position.x, 2) +
                           pow(pose_next_time_step.position.y - planes_.at(FRIENDLY_KEY).pose.position.y, 2));

    std::cout << "bogie at " << pose_next_time_step.position.x << " " << pose_next_time_step.position.y
              << " with distance " << distance << std::endl;

    double angle_to_friendly = atan2(planes_.at(FRIENDLY_KEY).pose.position.x - pose_next_time_step.position.x,
                                     planes_.at(FRIENDLY_KEY).pose.position.y - pose_next_time_step.position.y);

    double converted_orientation = atan2(sin(pose_next_time_step.orientation), cos(pose_next_time_step.orientation));

    double angle_error_bogie_to_friendly = fabs(angle_to_friendly - converted_orientation);

    if (angle_error_bogie_to_friendly > M_PI)
    {
      angle_error_bogie_to_friendly = 2 * M_PI - angle_error_bogie_to_friendly;
    }

    // double weight = pow(distance, angle_error_bogie_to_friendly);
    double weight = distance;
    addEdge(FRIENDLY_KEY, planes_key, weight);
  }

  unsigned int index_to_most_efficient_bogie = 1;
  for (int i = 1; i != weightedGraph_.at(FRIENDLY_KEY).size(); i++)
  {
    std::cout << "ping" << std::endl;
    if (weightedGraph_.at(FRIENDLY_KEY).at(i) < weightedGraph_.at(FRIENDLY_KEY).at(index_to_most_efficient_bogie))
    {
      index_to_most_efficient_bogie = i;
    }
  }

  double distance = sqrt(
      pow(planes_.at(FRIENDLY_KEY).pose.position.x - planes_.at(index_to_most_efficient_bogie).pose.position.x, 2) +
      pow(planes_.at(FRIENDLY_KEY).pose.position.y - planes_.at(index_to_most_efficient_bogie).pose.position.y, 2));

  std::cout << "bogie chosen at " << planes_.at(index_to_most_efficient_bogie).pose.position.x << " "
            << planes_.at(index_to_most_efficient_bogie).pose.position.y << " distance of " << distance << std::endl;
  std::cout << std::endl;

  poses.push_back(planes_.at(FRIENDLY_KEY).pose);
  // std::cout << poses.size() << std::endl;
  sim_->testPose(poses);

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
