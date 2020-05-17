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
      lock, [this]() { return std::chrono::duration<double, std::milli>().count() > FRIENDLY_UPDATE_PERIOD_MS; });
  range_bearings_from_friendly_.push_back(simulator_->rangeBearingToBogiesFromFriendly());
  range_bearings_from_friendly_.resize(DATA_SAMPLES_TO_TRACK);
  lock.unlock();
}

void Estimator::updateDataFromTower_()
{
  std::unique_lock<std::mutex> lock(tower_mx_);
  tower_condvar_.wait(
      lock, [this]() { return std::chrono::duration<double, std::milli>().count() > TOWER_UPDATE_PERIOD_MS; });
  range_velocity_from_tower_.push_back(simulator_->rangeVelocityToBogiesFromBase());
  range_velocity_from_tower_.resize(DATA_SAMPLES_TO_TRACK);
  lock.unlock();
}

std::vector<RangeBearingStamped>
Estimator::interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match)
{
  std::unique_lock<std::mutex> lock(friendly_mx_);
  std::vector<RangeBearingStamped> time1;
  std::vector<RangeBearingStamped> time2;
  for (auto sample_itr = range_bearings_from_friendly_.begin(); sample_itr != (range_bearings_from_friendly_.end() - 1);
       ++sample_itr)
  {
    if ((sample_to_match.front().timestamp >= (*sample_itr).front().timestamp) &&
        (sample_to_match.front().timestamp <= (*(sample_itr + 1)).front().timestamp))
    {
      time1 = *sample_itr;
      time2 = *(sample_itr + 1);
      break;
    }
  }
  std::vector<RangeBearingStamped> matched_data(sample_to_match.size());
  lock.unlock();

  for (int i = 0; i < matched_data.size(); i++)
  {
    matched_data.at(i) =
        RangeBearingStamped{ interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).range,
                                         time2.at(i).timestamp, time2.at(i).range),
                             interpolate(sample_to_match.at(i).timestamp, time1.at(i).timestamp, time1.at(i).bearing,
                                         time2.at(i).timestamp, time2.at(i).bearing),
                             (sample_to_match.at(i).timestamp) };
  }

  return matched_data;
}

double Estimator::interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2)
{
  return (((y2 - y1) / (x2 - x1)) * (x - x1) + y1);
}
