#ifndef DATAUPDATER_H
#define DATAUPDATER_H

#include "simulator.h"

class DataUpdater
{
private:
  /* data */
  std::shared_ptr<Simulator> simulator_;

  std::deque<std::vector<RangeBearingStamped>> range_bearings_from_friendly_;
  std::deque<std::vector<RangeVelocityStamped>> range_velocity_from_tower_;
  std::deque<Pose> poses_of_friendly_;

  std::mutex friendly_mx_;
  std::mutex tower_mx_;

public:
  DataUpdater(std::shared_ptr<Simulator> sim);
  ~DataUpdater();

  void updateDataFromFriendly();
  void updateDataFromTower();

  std::deque<std::vector<RangeBearingStamped>> getRangeBearingData();
  std::deque<std::vector<RangeVelocityStamped>> getRangeVelocityData();
  std::deque<Pose> getFriendlyPoseData();

  const unsigned int TOWER_DATA_SAMPLES_TO_TRACK = 3;
  const unsigned int FRIENDLY_DATA_SAMPLES_TO_TRACK = 25;
};

#endif