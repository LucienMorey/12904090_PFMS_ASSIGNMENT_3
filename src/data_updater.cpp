#include "data_updater.h"

DataUpdater::DataUpdater(Simulator* sim) : simulator_(sim)
{
}

DataUpdater::~DataUpdater()
{
}

void DataUpdater::updateDataFromFriendly(std::condition_variable* cv)
{
  while (1)
  {
    // copy both sim data sets to limit lock time
    auto temp_range_bearing = simulator_->rangeBearingToBogiesFromFriendly();
    auto temp_friendly_pose = simulator_->getFriendlyPose();

    // lock sample variables and record most recent samples
    std::unique_lock<std::mutex> lock(friendly_mx_);
    range_bearings_from_friendly_.push_front(temp_range_bearing);
    poses_of_friendly_.push_front(temp_friendly_pose);

    // constrain data set container to 3 and remove oldest data
    if (range_bearings_from_friendly_.size() > FRIENDLY_DATA_SAMPLES_TO_TRACK)
    {
      range_bearings_from_friendly_.resize(FRIENDLY_DATA_SAMPLES_TO_TRACK);
      poses_of_friendly_.resize(FRIENDLY_DATA_SAMPLES_TO_TRACK);
    }
    lock.unlock();
    cv->notify_one();
  }
}

void DataUpdater::updateDataFromTower(std::condition_variable* cv)
{
  while (1)
  {
    // copy range velocity data sample to limit lock time
    std::vector<RangeVelocityStamped> temp_velocity_range = simulator_->rangeVelocityToBogiesFromBase();

    // lock sample and record most recent sample
    std::unique_lock<std::mutex> lock(tower_mx_);
    range_velocity_from_tower_ = temp_velocity_range;
    lock.unlock();
    cv->notify_one();
  }
}

std::deque<std::vector<RangeBearingStamped>> DataUpdater::getRangeBearingData()
{
  // lock current sample so it cant be overwritten
  std::lock_guard<std::mutex> lock(friendly_mx_);
  return range_bearings_from_friendly_;
}

std::vector<RangeVelocityStamped> DataUpdater::getRangeVelocityData()
{
  // lock current sample so it cant be overwritten
  std::lock_guard<std::mutex> lock(tower_mx_);
  return range_velocity_from_tower_;
}

std::deque<Pose> DataUpdater::getFriendlyPoseData()
{
  // lock current sample so it cant be overwritten
  std::lock_guard<std::mutex> lock(tower_mx_);
  return poses_of_friendly_;
}