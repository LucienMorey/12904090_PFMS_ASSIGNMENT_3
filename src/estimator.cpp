#include "estimator.h"

Estimator::Estimator(Simulator* simulator)
{
  simulator_ = simulator;
}

Estimator::~Estimator()
{
}

void Estimator::updateDataFromFriendly_()
{
  std::unique_lock<std::mutex> lock(friendly_mx_);
  friendly_condvar_.wait(
      lock, [this]() { return std::chrono::duration<double, std::milli>().count() > FRIENDLY_UPDATE_PERIOD; });
  range_bearings_from_friendly_.push_back(simulator_->rangeBearingToBogiesFromFriendly());
  range_bearings_from_friendly_.resize(DATA_SAMPLES_TO_TRACK);
  lock.unlock();
}

void Estimator::updateDataFromTower_()
{
  std::unique_lock<std::mutex> lock(tower_mx_);
  tower_condvar_.wait(lock,
                      [this]() { return std::chrono::duration<double, std::milli>().count() > TOWER_UPDATE_PERIOD; });
  range_velocity_from_tower_.push_back(simulator_->rangeVelocityToBogiesFromBase());
  range_velocity_from_tower_.resize(DATA_SAMPLES_TO_TRACK);
  lock.unlock();
}
