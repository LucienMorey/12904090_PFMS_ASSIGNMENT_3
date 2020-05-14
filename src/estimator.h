#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "simulator.h"
#include "types.h"
#include <chrono>
#include <condition_variable>

class Estimator
{
private:
  /* data */
  Simulator* simulator_;
  void updateDataFromFriendly_();
  void updateDataFromTower_();

  std::vector<RangeBearingStamped> range_bearings_from_friendly_;
  std::vector<RangeVelocityStamped> range_velocity_from_tower_;

  std::vector<RangeBearingStamped> range_bearings_from_friendly_old_;
  std::vector<RangeVelocityStamped> range_velocity_from_tower_old_;

  std::mutex friendly_mx_;
  std::condition_variable friendly_condvar_;
  std::mutex tower_mx_;
  std::condition_variable tower_condvar_;

  AircraftContainer bogies;

  void findBogies_();

  const double FRIENDLY_UPDATE_PERIOD = 10.0;
  const double TOWER_UPDATE_PERIOD = 100.0;

public:
  Estimator(Simulator* simulator);
  ~Estimator();

  std::vector<Aircraft> getBogies();
};

#endif