#include "estimator.h"
#include <iostream>
Estimator::Estimator(const std::shared_ptr<Simulator>& simulator) : simulator_(simulator.get())
{
  updater = new DataUpdater(simulator_);

  friendly_updater = std::thread(&DataUpdater::updateDataFromFriendly, updater);
  base_updater = std::thread(&DataUpdater::updateDataFromTower, updater);
  // bogie_estimator = std::thread(&Estimator::determineBogies_, this);
}

Estimator::~Estimator()
{
  friendly_updater.join();
  base_updater.join();
  bogie_estimator.join();

  delete updater;
}

std::vector<RangeBearingStamped>
Estimator::interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match)
{
  // std::vector<RangeBearingStamped> time1;
  // std::vector<RangeBearingStamped> time2;
  // for (auto sample_itr = range_bearings_from_friendly_.begin(); sample_itr != (range_bearings_from_friendly_.end() -
  // 1);
  //      ++sample_itr)
  // {
  //   if ((sample_to_match.front().timestamp >= (*sample_itr).front().timestamp) &&
  //       (sample_to_match.front().timestamp <= (*(sample_itr + 1)).front().timestamp))
  //   {
  //     time1 = *sample_itr;
  //     time2 = *(sample_itr + 1);
  //     break;
  //   }
  //   else
  //   {
  //     time1 = *(range_bearings_from_friendly_.begin());
  //     time2 = *(range_bearings_from_friendly_.begin() + 1);
  //   }
  // }
  std::vector<RangeBearingStamped> matched_data(sample_to_match.size());

  // for (int i = 0; i < matched_data.size(); i++)
  // {
  //   matched_data.at(i) =
  //       RangeBearingStamped{ interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).range,
  //                                        time2.at(i).timestamp, time2.at(i).range),
  //                            interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).bearing,
  //                                        time2.at(i).timestamp, time2.at(i).bearing),
  //                            (sample_to_match.at(i).timestamp) };
  // }

  return matched_data;
}

double Estimator::interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2)
{
  return (((y2 - y1) / (x2 - x1)) * (x - x1) + y1);
}

void Estimator::determineBogies_()
{
  // while (range_velocity_from_tower_.size() < 3)
  //   ;
  // while (1)
  // {
  //   std::unique_lock<std::mutex> tower_lock(tower_mx_);

  //   std::deque<std::vector<RangeVelocityStamped>> range_velocity = range_velocity_from_tower_;
  //   std::deque<Pose> friendly_poses = poses_of_friendly_;

  //   // std::cout << range_velocity.size() << " " << range_velocity_from_tower_.size() << std::endl;

  //   tower_lock.unlock();

  //   std::deque<std::vector<RangeBearingStamped>> range_bearing;
  //   std::unique_lock<std::mutex> friendly_lock(friendly_mx_);

  //   for (auto samples : range_velocity)
  //   {
  //     range_bearing.push_back(interpolateRangeBearingData(samples));
  //   }

  //   // std::cout << range_bearing.at(0).size() << std::endl;
  //   friendly_lock.unlock();

  //   std::vector<std::vector<Aircraft>> bogies;
  //   for (int i = 0; i < range_velocity.size(); i++)
  //   {
  //     bogies.push_back(triangulateBogies(range_bearing.at(i), range_velocity.at(i), friendly_poses.at(i)));
  //   }

  //   std::vector<Aircraft> bogies_with_heading;

  //   for (auto newest_samples : bogies.front())
  //   {
  //     for (auto oldest_samples : bogies.back())
  //     {
  //       if (newest_samples.linear_velocity == oldest_samples.linear_velocity)
  //       {
  //         if (sqrt(pow(oldest_samples.pose.position.y - newest_samples.pose.position.y, 2) +
  //                  pow(oldest_samples.pose.position.x - newest_samples.pose.position.x, 2)) ==
  //             oldest_samples.linear_velocity *
  //                 (range_velocity.back().front().timestamp - range_velocity.front().front().timestamp))
  //         {
  //           for (auto old_samples : *(bogies.begin() + 1))
  //           {
  //             if ((old_samples.pose.position.x ==
  //                  ((newest_samples.pose.position.x + oldest_samples.pose.position.x) / 2)) &&
  //                 (old_samples.pose.position.y ==
  //                  ((newest_samples.pose.position.y + oldest_samples.pose.position.y) / 2)))
  //             {
  //               Pose bogie_pose = { newest_samples.pose.position,
  //                                   atan2((newest_samples.pose.position.y - oldest_samples.pose.position.y),
  //                                         (newest_samples.pose.position.x - oldest_samples.pose.position.x)) +
  //                                       2 * M_PI };
  //               Aircraft bogie;
  //               bogie.pose = bogie_pose;
  //               bogie.linear_velocity = newest_samples.linear_velocity;
  //               bogies_with_heading.push_back(bogie);
  //               break;
  //             }
  //           }
  //         }
  //         break;
  //       }
  //     }
  //   }

  //   std::lock_guard<std::mutex> aircraft_lock(bogies_.access);
  //   bogies_.a = bogies_with_heading;
  // }
}

std::vector<Aircraft> Estimator::triangulateBogies(std::vector<RangeBearingStamped> friendly_data,
                                                   std::vector<RangeVelocityStamped> base_data, Pose friendly_pose)
{
  std::vector<Aircraft> bogies;
  for (auto velocity_sample : base_data)
  {
    for (auto bearing_sample : friendly_data)
    {
      Aircraft bogie;
      bogie.pose = Pose{ transformBogietoGlobal(friendly_pose, bearing_sample), INFINITY };
      if (velocity_sample.range == sqrt(pow(bogie.pose.position.x, 2) + pow(bogie.pose.position.y, 2)))
      {
        bogie.linear_velocity = velocity_sample.velocity;
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

GlobalOrd Estimator::transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos)
{
  return GlobalOrd{
    friendly_pose.position.x + relative_pos.range * cos(relative_pos.bearing + friendly_pose.orientation),
    friendly_pose.position.y + relative_pos.range * sin(relative_pos.bearing + friendly_pose.orientation)
  };
}