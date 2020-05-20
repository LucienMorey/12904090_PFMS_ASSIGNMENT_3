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
  /* data */

  DataUpdater* updater;

  void determineBogies_();

  std::vector<Aircraft> matchBogies(std::vector<GlobalOrdStamped> range_bogies_global_t1,
                                    std::vector<GlobalOrdStamped> range_bogies_global_t2,
                                    std::vector<GlobalOrdStamped> range_bogies_global_t3);

  GlobalOrd transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos);

  std::thread friendly_updater;
  std::thread bogie_estimator;

  AircraftContainer bogies_;

  void findBogies_();

  const unsigned int CURRENT_INDEX = 0;
  const unsigned int OLD_INDEX = 1;
  const unsigned int OLDEST_INDEX = 2;

public:
  Estimator();
  ~Estimator();

  void setSimulator(const std::shared_ptr<Simulator>& simulator);

  std::vector<Aircraft> getBogies();
};

#endif