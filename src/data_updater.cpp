#include "data_updater.h"

DataUpdater::DataUpdater(std::shared_ptr<Simulator> sim) : simulator_(sim.get())
{
}

DataUpdater::~DataUpdater()
{
}

void DataUpdater::updateDataFromFriendly()
{
  while (1)
  {
    auto temp = simulator_->rangeBearingToBogiesFromFriendly();

    std::lock_guard<std::mutex> lock(friendly_mx_);
    range_bearings_from_friendly_.push_front(temp);
    if (range_bearings_from_friendly_.size() > FRIENDLY_DATA_SAMPLES_TO_TRACK)
    {
      range_bearings_from_friendly_.resize(FRIENDLY_DATA_SAMPLES_TO_TRACK);
    }
  }
}

void DataUpdater::updateDataFromTower()
{
  while (1)
  {
    auto temp_friendly_pose = simulator_->getFriendlyPose();
    auto temp_velocity_range = simulator_->rangeVelocityToBogiesFromBase();

    std::lock_guard<std::mutex> lock(tower_mx_);
    range_velocity_from_tower_.push_front(temp_velocity_range);
    poses_of_friendly_.push_front(temp_friendly_pose);
    if (range_velocity_from_tower_.size() > TOWER_DATA_SAMPLES_TO_TRACK)
    {
      range_velocity_from_tower_.resize(TOWER_DATA_SAMPLES_TO_TRACK);
      poses_of_friendly_.resize(TOWER_DATA_SAMPLES_TO_TRACK);
    }
  }
}

std::deque<std::vector<RangeBearingStamped>> DataUpdater::getRangeBearingData()
{
  std::lock_guard<std::mutex> lock(friendly_mx_);
  return range_bearings_from_friendly_;
}

std::deque<std::vector<RangeVelocityStamped>> DataUpdater::getRangeVelocityData()
{
  std::lock_guard<std::mutex> lock(tower_mx_);
  return range_velocity_from_tower_;
}

std::deque<Pose> DataUpdater::getFriendlyPoseData()
{
  std::lock_guard<std::mutex> lock(tower_mx_);
  return poses_of_friendly_;
}