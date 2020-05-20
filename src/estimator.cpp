#include "estimator.h"
#include <iostream>
Estimator::Estimator()
{
}

Estimator::~Estimator()
{
  friendly_updater.join();
  bogie_estimator.join();

  delete updater;
}

void Estimator::setSimulator(const std::shared_ptr<Simulator>& simulator)
{
  // set simulator for updater use
  updater = new DataUpdater(simulator);

  // now that simulator has been set threads can be created and estimation can begin
  friendly_updater = std::thread(&DataUpdater::updateDataFromFriendly, updater);
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
  while (updater->getRangeBearingData().size() < updater->FRIENDLY_DATA_SAMPLES_TO_TRACK)
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // loop continuously once minimum readings met
  while (1)
  {
    // get stored range_bearing data from bogie
    std::deque<std::vector<RangeBearingStamped>> range_bearing = updater->getRangeBearingData();
    std::deque<Pose> friendly_poses = updater->getFriendlyPoseData();

    // transform obtained range_bearing_data to position_data
    std::vector<GlobalOrdStamped> range_bogies_global_t1;

    // newest position data
    for (auto range_bearing_data : range_bearing.at(CURRENT_TIME_INDEX))
    {
      range_bogies_global_t1.push_back(
          transformBogietoGlobal(friendly_poses.at(CURRENT_TIME_INDEX), range_bearing_data));
    }

    // old position data
    std::vector<GlobalOrdStamped> range_bogies_global_t2;

    for (auto range_bearing_data : range_bearing.at(OLD_TIME_INDEX))
    {
      range_bogies_global_t2.push_back(transformBogietoGlobal(friendly_poses.at(OLD_TIME_INDEX), range_bearing_data));
    }

    // oldest position data
    std::vector<GlobalOrdStamped> range_bogies_global_t3;

    for (auto range_bearing_data : range_bearing.at(OLDEST_TIME_INDEX))
    {
      range_bogies_global_t3.push_back(
          transformBogietoGlobal(friendly_poses.at(OLDEST_TIME_INDEX), range_bearing_data));
    }

    // using the triangulated data match position and velocity to determine full pose data
    std::vector<Aircraft> bogies_with_heading =
        matchBogies(range_bogies_global_t1, range_bogies_global_t2, range_bogies_global_t3);

    // lock private member bogies for threadsafe operation
    std::lock_guard<std::mutex> aircraft_lock(bogies_.access);
    bogies_.a = bogies_with_heading;
  }
}

std::vector<Aircraft> Estimator::matchBogies(std::vector<GlobalOrdStamped> range_bogies_global_t1,
                                             std::vector<GlobalOrdStamped> range_bogies_global_t2,
                                             std::vector<GlobalOrdStamped> range_bogies_global_t3)
{
  std::vector<Aircraft> matched_bogies;
  int last_size = 0;

  // for each current sample loop through all the oldest samples
  for (auto current_sample : range_bogies_global_t1)
  {
    for (auto oldest_sample : range_bogies_global_t3)
    {
      for (auto old_sample : range_bogies_global_t2)
      {
        double heading_3_to_2 =
            atan2(old_sample.position.y - oldest_sample.position.y, old_sample.position.x - oldest_sample.position.x);
        double heading_2_to_1 =
            atan2(current_sample.position.y - old_sample.position.y, current_sample.position.x - old_sample.position.x);

        double heading_error = fabs(heading_2_to_1 - heading_3_to_2);
        if (heading_error > M_PI)
        {
          heading_error = 2 * M_PI - heading_error;
        }

        // calculate velocity betwween p3 and p2 && p2 p1
        double distance_3_to_2 = sqrt(pow(old_sample.position.x - oldest_sample.position.x, 2) +
                                      pow(old_sample.position.y - oldest_sample.position.y, 2));

        double distance_2_to_1 = sqrt(pow(current_sample.position.x - old_sample.position.x, 2) +
                                      pow(current_sample.position.y - old_sample.position.y, 2));

        // if p3 to p2 matches p3 to p1 && p2 p1 mathces p3 to p1
        if ((heading_error < M_PI / 18) && (fabs(distance_3_to_2 - distance_2_to_1) < 450))
        {
          // calculate an constrain bogie heading to 0-2pi
          double bogie_heading_1 = fmod(atan2((current_sample.position.y - old_sample.position.y),
                                              (current_sample.position.x - old_sample.position.x)) +
                                            2 * M_PI,
                                        2 * M_PI);

          // create temp pose for bogie
          Pose bogie_pose = { current_sample.position, bogie_heading_1 };

          // calculate velocity p3 to p1
          double distance_3_to_1 = sqrt(pow(current_sample.position.x - oldest_sample.position.x, 2) +
                                        pow(current_sample.position.y - oldest_sample.position.y, 2));

          // timestamps are in ms so must be converted to s
          double time_dif_3_to_1 = (current_sample.timestamp - oldest_sample.timestamp) / 1000.0;
          double velocity_from_3_to_1 = distance_3_to_1 / time_dif_3_to_1;

          // create and pushback matched bogie to list of bogies
          Aircraft bogie;
          bogie.pose = bogie_pose;
          bogie.linear_velocity = velocity_from_3_to_1;
          matched_bogies.push_back(bogie);
          break;
        }
      }
      // if found a match abandon curent new sample and match for next new sample
      if (matched_bogies.size() > last_size)
      {
        last_size++;
        break;
      }
    }
  }

  return matched_bogies;
}

GlobalOrdStamped Estimator::transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos)
{
  // return global co-ords of bogie relative to friendly

  GlobalOrdStamped global = {
    { friendly_pose.position.x + relative_pos.range * cos(relative_pos.bearing + friendly_pose.orientation),
      friendly_pose.position.y + relative_pos.range * sin(relative_pos.bearing + friendly_pose.orientation) },
    relative_pos.timestamp
  };

  return global;
}
