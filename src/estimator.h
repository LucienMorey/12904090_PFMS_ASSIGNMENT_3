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

  std::vector<RangeBearingStamped>
  interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match,
                              std::deque<std::vector<RangeBearingStamped>> data_to_interp);

  std::vector<Aircraft> triangulateBogies(std::vector<RangeBearingStamped> friendly_data,
                                          std::vector<RangeVelocityStamped> base_data, Pose friendly_pose);

  std::vector<Aircraft> matchBogies(std::vector<Aircraft> bogies_t1, std::vector<Aircraft> bogies_t2,
                                    std::vector<Aircraft> bogies_t3);

  double interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2);

  GlobalOrd transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos);

  std::thread friendly_updater;
  std::thread base_updater;
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