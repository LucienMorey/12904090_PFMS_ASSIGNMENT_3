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
  // set simulator for updater use
  updater = new DataUpdater(simulator);

  // now that simulator has been set threads can be created and estimation can begin
  friendly_updater = std::thread(&DataUpdater::updateDataFromFriendly, updater);
  base_updater = std::thread(&DataUpdater::updateDataFromTower, updater);
  bogie_estimator = std::thread(&Estimator::determineBogies_, this);
}

std::vector<Aircraft> Estimator::getBogies()
{
  // threadsafe getter for current matched bogies
  std::lock_guard<std::mutex> aircraft_lock(bogies_.access);
  return bogies_.a;
}

void Estimator::determineBogies_()
{
  // sleep until minimum readings reached
  while (updater->getRangeVelocityData().size() < updater->TOWER_DATA_SAMPLES_TO_TRACK)
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // loop continuously once minimum readings met
  while (1)
  {
    // slow down thread to avoid constant locking of mutexes
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // get stored range_velocity data from base
    std::deque<std::vector<RangeVelocityStamped>> range_velocity = updater->getRangeVelocityData();
    // get matching friendly poses
    std::deque<Pose> friendly_poses = updater->getFriendlyPoseData();

    // get stored range_bearing data from bogie
    std::deque<std::vector<RangeBearingStamped>> range_bearing = updater->getRangeBearingData();
    // create empty deque for interpolated bearing data
    std::deque<std::vector<RangeBearingStamped>> matched_range_bearing;

    // interpolate range_bearing data for each range velocity reading
    for (auto samples : range_velocity)
    {
      matched_range_bearing.push_back(interpolateRangeBearingData(samples, range_bearing));
    }

    // triangulate bogie Position for last 3 time steps
    std::vector<Aircraft> bogies_t1 = triangulateBogies(
        matched_range_bearing.at(CURRENT_INDEX), range_velocity.at(CURRENT_INDEX), friendly_poses.at(CURRENT_INDEX));

    std::vector<Aircraft> bogies_t2 = triangulateBogies(matched_range_bearing.at(OLD_INDEX),
                                                        range_velocity.at(OLD_INDEX), friendly_poses.at(OLD_INDEX));
    std::vector<Aircraft> bogies_t3 = triangulateBogies(
        matched_range_bearing.at(OLDEST_INDEX), range_velocity.at(OLDEST_INDEX), friendly_poses.at(OLDEST_INDEX));

    // using the triangulated data match position and velocity to determine full pose data
    std::vector<Aircraft> bogies_with_heading = matchBogies(bogies_t1, bogies_t2, bogies_t3);

    // lock private member bogies for threadsafe operation
    std::lock_guard<std::mutex> aircraft_lock(bogies_.access);
    bogies_.a = bogies_with_heading;
  }
}

std::vector<RangeBearingStamped>
Estimator::interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match,
                                       std::deque<std::vector<RangeBearingStamped>> data_for_interp)
{
  // create temp range bearing stamps to capture point before and after timestamp
  std::vector<RangeBearingStamped> time1;
  std::vector<RangeBearingStamped> time2;

  // find time before and after sample to match timestamp in range_bearing samples

  for (auto sample_itr = data_for_interp.begin(); sample_itr != (data_for_interp.end()); ++sample_itr)
  {
    // if reached end of data then to avoid out of bounds check set most recent data for extrapolation instead
    if (sample_itr == (data_for_interp.end() - 1))
    {
      time1 = *(data_for_interp.begin());
      time2 = *(data_for_interp.begin() + 1);
      // loop can break once boundaries for interpolation are found
      break;
    }
    else  // check if sample ts is between itr ts and next itr timestamp for interpolation
    {
      if ((sample_to_match.front().timestamp >= (*sample_itr).front().timestamp) &&
          (sample_to_match.front().timestamp <= (*(sample_itr + 1)).front().timestamp))
      {
        time1 = *sample_itr;
        time2 = *(sample_itr + 1);
        // loop can be broken once boundaries for interpoilation are found
        break;
      }
    }
  }
  // create vector of the same size as the data to match
  std::vector<RangeBearingStamped> matched_data(sample_to_match.size());

  // for length of data interpolate to match range, bearing and ts
  for (int i = 0; i < matched_data.size(); i++)
  {
    matched_data.at(i) =
        RangeBearingStamped{ interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).range,
                                         time2.at(i).timestamp, time2.at(i).range),
                             interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).bearing,
                                         time2.at(i).timestamp, time2.at(i).bearing),
                             (sample_to_match.at(i).timestamp) };
  }

  // return the matched data
  return matched_data;
}

std::vector<Aircraft> Estimator::triangulateBogies(std::vector<RangeBearingStamped> friendly_data,
                                                   std::vector<RangeVelocityStamped> base_data, Pose friendly_pose)
{
  std::vector<Aircraft> bogies;

  // for each rangebearing reading transform it to global co-ords
  for (auto bearing_sample : friendly_data)
  {
    Aircraft bogie;
    bogie.pose = Pose{ transformBogietoGlobal(friendly_pose, bearing_sample), 0.0 };

    // find the matching velocity data
    for (auto velocity_sample : base_data)
    {
      // if found matching data then set bogie velocity
      if (fabs(velocity_sample.range - sqrt(pow(bogie.pose.position.x, 2) + pow(bogie.pose.position.y, 2))) < 100.0)
      {
        bogie.linear_velocity = velocity_sample.velocity;
        bogies.push_back(bogie);
        // break once found matching data for next sample
        break;
      }
    }
  }

  return bogies;
}

std::vector<Aircraft> Estimator::matchBogies(std::vector<Aircraft> bogies_t1, std::vector<Aircraft> bogies_t2,
                                             std::vector<Aircraft> bogies_t3)
{
  std::vector<Aircraft> matched_bogies;

  // for each current sample loop through all the oldest samples
  for (auto current_sample : bogies_t1)
  {
    for (auto oldest_sample : bogies_t3)
    {
      // if the velocity of the the oldest and current samples match check for state of third value
      if (fabs(current_sample.linear_velocity - oldest_sample.linear_velocity) < 0.1)
      {
        for (auto old_sample : bogies_t2)
        {
          // point lies within square projected out by line created with oldest and current data

          if ((fabs(((current_sample.pose.position.x + oldest_sample.pose.position.x) / 2.0 -
                     old_sample.pose.position.x) < 100.0)) &&
              (fabs(((current_sample.pose.position.y + oldest_sample.pose.position.y) / 2.0 -
                     old_sample.pose.position.y) < 100.0)))
          {
            // calculate an constrain bogie heading to 0-2pi
            double bogie_heading =
                fmod(atan2((current_sample.pose.position.y - oldest_sample.pose.position.y),
                           (current_sample.pose.position.x - oldest_sample.pose.position.x) + 2.0 * M_PI),
                     2.0 * M_PI);

            // create temp pose for bogie
            Pose bogie_pose = { current_sample.pose.position, bogie_heading };

            // create and pushback matched bogie to list of bogies
            Aircraft bogie;
            bogie.pose = bogie_pose;
            bogie.linear_velocity = current_sample.linear_velocity;
            matched_bogies.push_back(bogie);
            break;
          }
        }
        break;
      }
    }
  }

  return matched_bogies;
}

GlobalOrd Estimator::transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos)
{
  // return global co-ords of bogie relative to friendly
  return GlobalOrd{
    friendly_pose.position.x + relative_pos.range * cos(relative_pos.bearing + friendly_pose.orientation),
    friendly_pose.position.y + relative_pos.range * sin(relative_pos.bearing + friendly_pose.orientation)
  };
}

double Estimator::interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2)
{
  // return interpolated point
  return (((y2 - y1) / (x2 - x1)) * (x - x1) + y1);
}