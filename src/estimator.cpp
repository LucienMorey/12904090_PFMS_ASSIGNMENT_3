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
    // slow down thread to avoid constant locking of mutexes
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // get stored range_bearing data from bogie
    std::deque<std::vector<RangeBearingStamped>> range_bearing = updater->getRangeBearingData();
    std::deque<Pose> friendly_poses = updater->getFriendlyPoseData();

    // transform to global
    std::vector<GlobalOrdStamped> range_bogies_global_t1;
    std::vector<GlobalOrdStamped> range_bogies_global_t2;
    std::vector<GlobalOrdStamped> range_bogies_global_t3;

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

  // for each current sample loop through all the oldest samples
  for (auto current_sample : range_bogies_global_t1)
  {
    for (auto oldest_sample : range_bogies_global_t3)
    {
      // calculate velocity p3 to p1

      for (auto old_sample : range_bogies_global_t2)
      {
        // calculate velocity betwween p3 and p2 && p2 p1

        // if p3 to p2 matches p3 to p1 && p2 p1 mathces p3 to p1

        // calculate an constrain bogie heading to 0-2pi
        // double bogie_heading =
        //     fmod(atan2((current_sample.pose.position.y - oldest_sample.pose.position.y),
        //                (current_sample.pose.position.x - oldest_sample.pose.position.x) + 2.0 * M_PI),
        //          2.0 * M_PI);

        // create temp pose for bogie
        // Pose bogie_pose = { current_sample.pose.position, bogie_heading };

        // create and pushback matched bogie to list of bogies
        Aircraft bogie;
        // bogie.pose = bogie_pose;
        // bogie.linear_velocity = current_sample.linear_velocity;
        // reset timer to sim time now - newest timestamp
        // matched_bogies.push_back(bogie);
        // break;
      }
    }

    // if size grew by 1
    // break
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
