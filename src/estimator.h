#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "simulator.h"
#include "types.h"
#include <chrono>
#include <condition_variable>
#include <utility>

class Estimator
{
private:
  /* data */
  std::shared_ptr<Simulator> simulator_;
  void updateDataFromFriendly_();
  void updateDataFromTower_();
  void determineBogies_();

  std::vector<RangeBearingStamped>
  interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match);

  std::vector<Aircraft> triangulateBogies(std::vector<RangeBearingStamped> friendly_data,
                                          std::vector<RangeVelocityStamped> base_data, Pose friendly_pose);

  double interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2);

  std::deque<std::vector<RangeBearingStamped>> range_bearings_from_friendly_;
  std::deque<std::vector<RangeVelocityStamped>> range_velocity_from_tower_;
  std::deque<Pose> poses_of_friendly_;

  std::mutex friendly_mx_;
  std::mutex tower_mx_;

  std::thread friendly_updater;
  std::thread base_updater;
  std::thread bogie_estimator;

  AircraftContainer bogies_;

  void findBogies_();

  const unsigned int TOWER_DATA_SAMPLES_TO_TRACK = 3;
  const unsigned int FRIENDLY_DATA_SAMPLES_TO_TRACK = 20;

public:
  Estimator(const std::shared_ptr<Simulator>& simulator);
  ~Estimator();

  std::vector<Aircraft> getBogies();
};

#endif