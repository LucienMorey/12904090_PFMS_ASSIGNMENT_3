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

  void determineBogies_();

  std::vector<Aircraft> matchBogies(std::vector<GlobalOrdStamped> range_bogies_global_t1,
                                    std::vector<GlobalOrdStamped> range_bogies_global_t2,
                                    std::vector<GlobalOrdStamped> range_bogies_global_t3);

  GlobalOrdStamped transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos);

  std::thread friendly_updater;
  std::thread bogie_estimator;
  std::thread base_updater;

  AircraftContainer bogies_;

  const unsigned int CURRENT_TIME_INDEX = 0;
  const unsigned int OLD_TIME_INDEX = 1;
  const unsigned int OLDEST_TIME_INDEX = 2;

  const double ANGLE_TOLERENCE = M_PI / 18;
  const double SEGMENT_LENGTH_TOLERENCE = 5.0;

  std::condition_variable friendly_cv;
  std::condition_variable base_cv;

  std::mutex friendly_mx_;
  std::mutex base_mx_;

public:
  Estimator();
  ~Estimator();

  void setSimulator(Simulator* simulator);

  std::vector<Aircraft> getBogies();
};

#endif