#include "data_updater.h"

/**
 * @brief Construct a new Data Updater:: Data Updater object. A simulator object will need to be created and spawned
 * before this object can be used so that data can be obtained
 *
 * @param sim Simulator to pull data readings from
 */
DataUpdater::DataUpdater(Simulator* sim) : simulator_(sim)
{
}

/**
 * @brief Destroy the Data Updater:: Data Updater object
 *
 */
DataUpdater::~DataUpdater()
{
}

/**
 * @brief A thread function that will obtain range bearing data from the aircraft to bogies and will update the friendly
 * pose at the corresponding timestamp anage how long samples are kept. The function It takes a condition variable
 * parameter that will be notified after a sample is successfully stored.
 *
 * @param cv - a condition variable that will have the notify_one function called after a data sample is obtained
 * sucessfully
 */
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

/**
 * @brief A thread function that will continuously obtain the latest Range velocity reading from the base station and
 * store it within the class. The function takes a condition variable that will recieve the notify one function after
 * the storing process has been completed sucessfuly
 *
 * @param cv - a condition variable that will have the notify_one function called after a data sample is obtained
 * sucessfully
 */
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

/**
 * @brief thread safe getter of most recent range bearing data samples from the friendly aircraft
 *
 * @return std::deque<std::vector<RangeBearingStamped>> - deque containing the most recent range bearing samples.
 */
std::deque<std::vector<RangeBearingStamped>> DataUpdater::getRangeBearingData()
{
  // lock current sample so it cant be overwritten
  std::lock_guard<std::mutex> lock(friendly_mx_);
  return range_bearings_from_friendly_;
}

/**
 * @brief Thread safe getter for the most recent range velocity sample from the base station
 *
 * @return std::vector<RangeVelocityStamped> - vector containing all rangeVelocity readings from the most recent range
 * velocity sample
 */
std::vector<RangeVelocityStamped> DataUpdater::getRangeVelocityData()
{
  // lock current sample so it cant be overwritten
  std::lock_guard<std::mutex> lock(tower_mx_);
  return range_velocity_from_tower_;
}

/**
 * @brief threadsafe getter of the most most recent friendly poses. Timestamps will match the timestamps from the
 * getRangeBearing data function
 *
 * @return std::deque<Pose> - deque of Most recent Poses
 */
std::deque<Pose> DataUpdater::getFriendlyPoseData()
{
  // lock current sample so it cant be overwritten
  std::lock_guard<std::mutex> lock(tower_mx_);
  return poses_of_friendly_;
}