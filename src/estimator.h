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

  std::vector<RangeBearingStamped>
  interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match);

  double interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2);

  std::vector<std::vector<RangeBearingStamped>> range_bearings_from_friendly_;
  std::vector<std::vector<RangeVelocityStamped>> range_velocity_from_tower_;

  std::mutex friendly_mx_;
  std::condition_variable friendly_condvar_;
  std::mutex tower_mx_;
  std::condition_variable tower_condvar_;

  AircraftContainer bogies;

  void findBogies_();

  const double FRIENDLY_UPDATE_PERIOD_MS = (1 / Simulator::FRIENDLY_REF_RATE) * 1000;
  const double TOWER_UPDATE_PERIOD_MS = (1 / Simulator::BSTATION_REF_RATE) * 1000;
  const unsigned int DATA_SAMPLES_TO_TRACK = 10;

public:
  Estimator(Simulator* simulator);
  ~Estimator();

  std::vector<Aircraft> getBogies();
};

#endif