#include "estimator.h"
#include <iostream>
Estimator::Estimator()
{
}

Estimator::~Estimator()
{
  friendly_updater.join();
  base_updater.join();
  bogie_estimator.join();

  delete updater;
}

void Estimator::setSimulator(const std::shared_ptr<Simulator>& simulator)
{
  simulator_ = simulator;
  updater = new DataUpdater(simulator_);

  friendly_updater = std::thread(&DataUpdater::updateDataFromFriendly, updater);
  base_updater = std::thread(&DataUpdater::updateDataFromTower, updater);
  bogie_estimator = std::thread(&Estimator::determineBogies_, this);
}

std::vector<RangeBearingStamped>
Estimator::interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match,
                                       std::deque<std::vector<RangeBearingStamped>> data_for_interp)
{
  std::vector<RangeBearingStamped> time1;
  std::vector<RangeBearingStamped> time2;
  for (auto sample_itr = data_for_interp.begin(); sample_itr != (data_for_interp.end() - 1); ++sample_itr)
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
      time1 = *(data_for_interp.begin());
      time2 = *(data_for_interp.begin() + 1);
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
  double y = (((y2 - y1) / (x2 - x1)) * (x - x1) + y1);
  return y;
}

void Estimator::determineBogies_()
{
  while (updater->getRangeVelocityData().size() < updater->TOWER_DATA_SAMPLES_TO_TRACK)
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

  while (1)
  {
    std::deque<std::vector<RangeVelocityStamped>> range_velocity = updater->getRangeVelocityData();
    std::deque<Pose> friendly_poses = updater->getFriendlyPoseData();

    std::deque<std::vector<RangeBearingStamped>> range_bearing = updater->getRangeBearingData();
    std::deque<std::vector<RangeBearingStamped>> matched_range_bearing;

    // std::deque<std::vector<RangeVelocityStamped>> range_velocity = { range_velocity_samples.at(0),
    //                                                                  range_velocity_samples.at(5),
    //                                                                  range_velocity_samples.at(10) };
    // std::deque<Pose> friendly_poses = { friendly_poses_samples.at(0), friendly_poses_samples.at(5),
    //                                     friendly_poses_samples.at(10) };

    for (auto samples : range_velocity)
    {
      matched_range_bearing.push_back(interpolateRangeBearingData(samples, range_bearing));
    }

    auto vel_itr = range_velocity.begin();
    auto bear_it = matched_range_bearing.begin();

    std::vector<std::pair<std::vector<Aircraft>, long>> bogies_over_time;
    for (int i = 0; i < range_velocity.size(); i++)
    {
      bogies_over_time.push_back(
          std::make_pair(triangulateBogies(range_bearing.at(i), range_velocity.at(i), friendly_poses.at(i)),
                         range_velocity.at(i).front().timestamp));
    }

    std::vector<Aircraft> bogies_with_heading = matchBogies(bogies_over_time);

    std::vector<Pose> bogie_pose;
    for (auto bogies : bogies_with_heading)
    {
      // for (auto bogie : bogies.first)
      //{
      bogie_pose.push_back(bogies.pose);
      //}
    }

    simulator_->testPose(bogie_pose);

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
      Aircraft bogie;
      bogie.pose = Pose{ transformBogietoGlobal(friendly_pose, bearing_sample), 0.0 };

      if (fabs(velocity_sample.range - sqrt(pow(bogie.pose.position.x, 2) + pow(bogie.pose.position.y, 2))) < 100.0)
      {
        bogie.linear_velocity = velocity_sample.velocity;
        bogies.push_back(bogie);
        // std::cout << "bogie_pose " << bogie.pose.position.x << ", " << bogie.pose.position.y << std::endl;

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

std::vector<Aircraft> Estimator::matchBogies(std::vector<std::pair<std::vector<Aircraft>, long>> bogies_over_time)
{
  std::vector<Aircraft> matched_bogies;

  for (auto newest_samples : bogies_over_time.front().first)
  {
    for (auto oldest_samples : bogies_over_time.back().first)
    {
      if (newest_samples.linear_velocity == oldest_samples.linear_velocity)
      {
        std::cout << "ping" << std::endl;
        // std::cout << fabs(sqrt(pow(oldest_samples.pose.position.y - newest_samples.pose.position.y, 2) +
        //                        pow(oldest_samples.pose.position.x - newest_samples.pose.position.x, 2)) -
        //                   oldest_samples.linear_velocity *
        //                       (bogies_over_time.back().second - bogies_over_time.front().second) / 1000)
        //           << std::endl;
        if (fabs(sqrt(pow(oldest_samples.pose.position.y - newest_samples.pose.position.y, 2) +
                      pow(oldest_samples.pose.position.x - newest_samples.pose.position.x, 2)) -
                 oldest_samples.linear_velocity * (bogies_over_time.back().second - bogies_over_time.front().second)) /
                1000 <
            100)
        {
          for (auto old_samples : (*(bogies_over_time.begin() + 1)).first)
          {
            // std::cout << "centre x " << old_samples.pose.position.x << " calculated centre x "
            //           << ((newest_samples.pose.position.x + oldest_samples.pose.position.x) / 2) << std::endl;

            // std::cout << "centre y " << old_samples.pose.position.y << " calculated centre y "
            //           << ((newest_samples.pose.position.y + oldest_samples.pose.position.y) / 2) << std::endl;

            if (fabs((old_samples.pose.position.x -
                      ((newest_samples.pose.position.x + oldest_samples.pose.position.x) / 2)) < 1.0) &&
                (fabs((old_samples.pose.position.y -
                       ((newest_samples.pose.position.y + oldest_samples.pose.position.y) / 2))) < 1.0))
            {
              // std::cout << "pang" << std::endl;

              double oldest_to_newest = atan2((newest_samples.pose.position.y - oldest_samples.pose.position.y),
                                              (newest_samples.pose.position.x - oldest_samples.pose.position.x));

              double old_to_newest = atan2((newest_samples.pose.position.y - old_samples.pose.position.y),
                                           (newest_samples.pose.position.x - old_samples.pose.position.x));

              double oldest_to_old = atan2((old_samples.pose.position.y - oldest_samples.pose.position.y),
                                           (old_samples.pose.position.x - oldest_samples.pose.position.x));
              double average_angle = (oldest_to_newest + oldest_to_old + old_to_newest) / 3;
              Pose bogie_pose = { newest_samples.pose.position, average_angle };

              if (bogie_pose.orientation < 0)
              {
                bogie_pose.orientation = M_PI + (M_PI - fabs(bogie_pose.orientation));
              }
              Aircraft bogie;
              bogie.pose = bogie_pose;
              bogie.linear_velocity = newest_samples.linear_velocity;
              matched_bogies.push_back(bogie);
              break;
            }
          }
        }
        break;
      }
    }
  }

  return matched_bogies;
}