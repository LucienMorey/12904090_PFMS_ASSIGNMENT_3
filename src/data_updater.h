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
  /**
   * @brief Construct a new Data Updater object. A simulator object will need to be created and spawned
   * before this object can be used so that data can be obtained
   *
   * @param sim Simulator to pull data readings from
   */
  DataUpdater(Simulator* sim);

  /**
   * @brief Destroy the Data Updater object
   *
   */
  ~DataUpdater();
  /**
   * @brief A thread function that will obtain range bearing data from the aircraft to bogies and will update the
   * friendly pose at the corresponding timestamp anage how long samples are kept. The function It takes a condition
   * variable parameter that will be notified after a sample is successfully stored.
   *
   * @param cv - a condition variable that will have the notify_one function called after a data sample is obtained
   * sucessfully
   */
  void updateDataFromFriendly(std::condition_variable* cv);

  /**
   * @brief A thread function that will continuously obtain the latest Range velocity reading from the base station and
   * store it within the class. The function takes a condition variable that will recieve the notify one function after
   * the storing process has been completed sucessfuly
   *
   * @param cv - a condition variable that will have the notify_one function called after a data sample is obtained
   * sucessfully
   */
  void updateDataFromTower(std::condition_variable* cv);

  /**
   * @brief thread safe getter of most recent range bearing data samples from the friendly aircraft
   *
   * @return std::deque<std::vector<RangeBearingStamped>> - deque containing the most recent range bearing samples.
   */
  std::deque<std::vector<RangeBearingStamped>> getRangeBearingData();

  /**
   * @brief Thread safe getter for the most recent range velocity sample from the base station
   *
   * @return std::vector<RangeVelocityStamped> - vector containing all rangeVelocity readings from the most recent range
   * velocity sample
   */
  std::vector<RangeVelocityStamped> getRangeVelocityData();

  /**
   * @brief threadsafe getter of the most most recent friendly poses. Timestamps will match the timestamps from the
   * getRangeBearing data function
   *
   * @return std::deque<Pose> - deque of Most recent Poses
   */
  std::deque<Pose> getFriendlyPoseData();

  const unsigned int FRIENDLY_DATA_SAMPLES_TO_TRACK = 3;
};

#endif