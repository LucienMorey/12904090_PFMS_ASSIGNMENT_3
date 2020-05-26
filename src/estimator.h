#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "simulator.h"
#include "types.h"
#include "data_updater.h"
#include <chrono>
#include <condition_variable>
#include <utility>

class Estimator
{
private:
  DataUpdater* updater;

  /**
   * @brief retrieve most recent sensor data and fuse it to triangulate bogies. There is no return but instead the
   * function is designed to run within a thread and constantly triangulate bogies in real time.
   *
   */
  void determineBogies_();

  /**
   * @brief Create 2 line segments between old/previous and previous/current data and compare them based on length, and
   * gradient to match bogies and determine heading and velocity in the current time step
   *
   * @param range_bogies_global_t1 GlobalOrdStamped - Global position of bogies current time step
   * @param range_bogies_global_t2 GlobalOrdStamped - Global position of bogies previous time step
   * @param range_bogies_global_t3 GlobalOrdStamped - Global position of bogies in old timestep
   * @return std::vector<Aircraft> Vector container containing all bogies matched in the current timestep in no
   * particular order
   */
  std::vector<Aircraft> matchBogies_(std::vector<GlobalOrdStamped> range_bogies_global_t1,
                                     std::vector<GlobalOrdStamped> range_bogies_global_t2,
                                     std::vector<GlobalOrdStamped> range_bogies_global_t3);

  /**
   * @brief Transform helper function. Transforms local range bearing data of friendly scan to global position and
   * assigns the position the timestamp of the data reading
   *
   * @param friendly_pose - Pose pose of friendly for use in transform
   * @param relative_pos - RangeBearingStamped struct of detected bogie relative to friendly aircraft
   * @return GlobalOrdStamped - global position of bogie with a timestamp in ms
   */
  GlobalOrdStamped transformBogietoGlobal_(Pose friendly_pose, RangeBearingStamped relative_pos);

  std::thread friendly_updater_;
  std::thread bogie_estimator_;
  std::thread base_updater_;

  AircraftContainer bogies_;

  const unsigned int CURRENT_TIME_INDEX = 0;
  const unsigned int OLD_TIME_INDEX = 1;
  const unsigned int OLDEST_TIME_INDEX = 2;

  const double ANGLE_TOLERENCE = M_PI / 18;
  const double SEGMENT_LENGTH_TOLERENCE = 5.0;

  std::condition_variable friendly_cv_;
  std::condition_variable base_cv_;

  std::mutex friendly_mx_;
  std::mutex base_mx_;

public:
  /**
   * @brief Construct a new Estimator object
   *
   */
  Estimator();

  /**
   * @brief Destroy the Estimator object. Joins any started estimation threads and cleans up pointer
   *
   */
  ~Estimator();

  /**
   * @brief set a simulator to pull data from. once a simulator has been set threads obtaining data and estimating bogie
   * pose and twist will begin
   *
   * @param simulator - Simulator pointer to be used for data retrieval
   */
  void setSimulator(Simulator* simulator);

  /**
   * @brief Thread safe getter for bogies that have been triangulated
   *
   * @return std::vector<Aircraft> - Vector contrainer of triangulated bogies
   */
  std::vector<Aircraft> getBogies();
};

#endif