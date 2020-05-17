#include "estimator.h"
#include <iostream>
Estimator::Estimator(const std::shared_ptr<Simulator>& simulator) : simulator_(simulator.get())
{
  friendly_updater = std::thread(&Estimator::updateDataFromFriendly_, this);
  base_updater = std::thread(&Estimator::updateDataFromTower_, this);
  bogie_estimator = std::thread(&Estimator::determineBogies_, this);
}

Estimator::~Estimator()
{
  friendly_updater.join();
  base_updater.join();
  bogie_estimator.join();
}

void Estimator::updateDataFromFriendly_()
{
  while (1)
  {
    auto temp = simulator_->rangeBearingToBogiesFromFriendly();

    std::lock_guard<std::mutex> lock(friendly_mx_);
    range_bearings_from_friendly_.push_front(temp);
    if (range_bearings_from_friendly_.size() > FRIENDLY_DATA_SAMPLES_TO_TRACK)
    {
      range_bearings_from_friendly_.resize(FRIENDLY_DATA_SAMPLES_TO_TRACK);
    }
  }
}

void Estimator::updateDataFromTower_()
{
  while (1)
  {
    auto temp_friendly_pose = simulator_->getFriendlyPose();
    auto temp_velocity_range = simulator_->rangeVelocityToBogiesFromBase();

    std::lock_guard<std::mutex> lock(tower_mx_);
    range_velocity_from_tower_.push_front(temp_velocity_range);
    poses_of_friendly_.push_front(temp_friendly_pose);
    if (range_velocity_from_tower_.size() > TOWER_DATA_SAMPLES_TO_TRACK)
    {
      range_velocity_from_tower_.resize(TOWER_DATA_SAMPLES_TO_TRACK);
      poses_of_friendly_.resize(TOWER_DATA_SAMPLES_TO_TRACK);
    }
  }
}

std::vector<RangeBearingStamped>
Estimator::interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match)
{
  std::vector<RangeBearingStamped> time1;
  std::vector<RangeBearingStamped> time2;
  for (auto sample_itr = range_bearings_from_friendly_.begin(); sample_itr != (range_bearings_from_friendly_.end() - 1);
       ++sample_itr)
  {
    if ((sample_to_match.front().timestamp >= (*sample_itr).front().timestamp) &&
        (sample_to_match.front().timestamp <= (*(sample_itr + 1)).front().timestamp))
    {
      time1 = *sample_itr;
      time2 = *(sample_itr + 1);
      break;
    }
    else
    {
      time1 = *(range_bearings_from_friendly_.begin());
      time2 = *(range_bearings_from_friendly_.begin() + 1);
    }
  }
  std::vector<RangeBearingStamped> matched_data(sample_to_match.size());

  for (int i = 0; i < matched_data.size(); i++)
  {
    matched_data.at(i) =
        RangeBearingStamped{ interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).range,
                                         time2.at(i).timestamp, time2.at(i).range),
                             interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).bearing,
                                         time2.at(i).timestamp, time2.at(i).bearing),
                             (sample_to_match.at(i).timestamp) };
  }

  return matched_data;
}

double Estimator::interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2)
{
  return (((y2 - y1) / (x2 - x1)) * (x - x1) + y1);
}

void Estimator::determineBogies_()
{
  while (range_velocity_from_tower_.size() < 3)
    ;
  while (1)
  {
    std::unique_lock<std::mutex> tower_lock(tower_mx_);

    std::deque<std::vector<RangeVelocityStamped>> range_velocity = range_velocity_from_tower_;
    std::deque<Pose> friendly_poses = poses_of_friendly_;

    // std::cout << range_velocity.size() << " " << range_velocity_from_tower_.size() << std::endl;

    tower_lock.unlock();

    std::deque<std::vector<RangeBearingStamped>> range_bearing;
    std::unique_lock<std::mutex> friendly_lock(friendly_mx_);

    for (auto samples : range_velocity)
    {
      range_bearing.push_back(interpolateRangeBearingData(samples));
    }

    // std::cout << range_bearing.at(0).size() << std::endl;
    friendly_lock.unlock();

    std::vector<std::vector<Aircraft>> bogies;
    for (int i = 0; i < range_velocity.size(); i++)
    {
      bogies.push_back(triangulateBogies(range_bearing.at(i), range_velocity.at(i), friendly_poses.at(i)));
    }

    std::vector<Aircraft> bogies_with_heading;

    for (auto newest_samples : bogies.front())
    {
      for (auto oldest_samples : bogies.back())
      {
        if (newest_samples.linear_velocity == oldest_samples.linear_velocity)
        {
          if (sqrt(pow(oldest_samples.pose.position.y - newest_samples.pose.position.y, 2) +
                   pow(oldest_samples.pose.position.x - newest_samples.pose.position.x, 2)) ==
              oldest_samples.linear_velocity *
                  (range_velocity.back().front().timestamp - range_velocity.front().front().timestamp))
          {
            for (auto old_samples : *(bogies.begin() + 1))
            {
              if ((old_samples.pose.position.x ==
                   ((newest_samples.pose.position.x + oldest_samples.pose.position.x) / 2)) &&
                  (old_samples.pose.position.y ==
                   ((newest_samples.pose.position.y + oldest_samples.pose.position.y) / 2)))
              {
                Pose bogie_pose = { newest_samples.pose.position,
                                    atan2((newest_samples.pose.position.y - oldest_samples.pose.position.y),
                                          (newest_samples.pose.position.x - oldest_samples.pose.position.x)) +
                                        2 * M_PI };
                Aircraft bogie;
                bogie.pose = bogie_pose;
                bogie.linear_velocity = newest_samples.linear_velocity;
                bogies_with_heading.push_back(bogie);
                break;
              }
            }
          }
          break;
        }
      }
    }

    std::lock_guard<std::mutex> aircraft_lock(bogies_.access);
    bogies_.a = bogies_with_heading;
  }
}

std::vector<Aircraft> Estimator::triangulateBogies(std::vector<RangeBearingStamped> friendly_data,
                                                   std::vector<RangeVelocityStamped> base_data, Pose friendly_pose)
{
  std::vector<Aircraft> bogies;
  for (auto velocity_sample : base_data)
  {
    for (auto bearing_sample : friendly_data)
    {
      if (velocity_sample.range ==
          sqrt(pow((friendly_pose.position.y +
                    bearing_sample.range * sin(friendly_pose.orientation + bearing_sample.bearing)),
                   2) +
               pow((friendly_pose.position.x +
                    bearing_sample.range * cos(friendly_pose.orientation + bearing_sample.bearing)),
                   2)))
      {
        Aircraft bogie;
        bogie.pose = Pose{ GlobalOrd{ (friendly_pose.position.x +
                                       bearing_sample.range * cos(friendly_pose.orientation + bearing_sample.bearing)),
                                      (friendly_pose.position.y + bearing_sample.range * sin(friendly_pose.orientation +
                                                                                             bearing_sample.bearing)) },
                           0.0 };
        bogie.linear_velocity = velocity_sample.velocity;
        // std::cout << "[" << simulator_->elapsed() / 1000 << "s]" << std::endl;
        // std::cout << "Friendly {x, y, orientation}:" << std::endl;
        // std::cout << "  - x: " << bogie.pose.position.x << "m" << std::endl;
        // std::cout << "  - y: " << bogie.pose.position.y << "m" << std::endl;
        // std::cout << "  - orient: " << bogie.pose.orientation * 180 / M_PI << " deg" << std::endl << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bogies.push_back(bogie);
        break;
      }
    }
  }

  return bogies;
}

std::vector<Aircraft> Estimator::getBogies()
{
  std::lock_guard<std::mutex> aircraft_lock(bogies_.access);
  return bogies_.a;
}