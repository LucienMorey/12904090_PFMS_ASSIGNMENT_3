#ifndef DATAUPDATER_H
#define DATAUPDATER_H

#include "simulator.h"
#include <condition_variable>

class DataUpdater
{
private:
  /* data */
  Simulator* simulator_;

  std::deque<std::vector<RangeBearingStamped>> range_bearings_from_friendly_;
  std::vector<RangeVelocityStamped> range_velocity_from_tower_;
  std::deque<Pose> poses_of_friendly_;

  std::mutex friendly_mx_;
  std::mutex tower_mx_;

public:
  DataUpdater(Simulator* sim);
  ~DataUpdater();

  void updateDataFromFriendly(std::condition_variable* cv);
  void updateDataFromTower(std::condition_variable* cv);

  std::deque<std::vector<RangeBearingStamped>> getRangeBearingData();
  std::vector<RangeVelocityStamped> getRangeVelocityData();
  std::deque<Pose> getFriendlyPoseData();

  const unsigned int FRIENDLY_DATA_SAMPLES_TO_TRACK = 3;
};

#endif