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
  std::shared_ptr<Simulator> simulator_;

  DataUpdater* updater;

  void determineBogies_();

  std::vector<RangeBearingStamped>
  interpolateRangeBearingData(const std::vector<RangeVelocityStamped>& sample_to_match,
                              std::deque<std::vector<RangeBearingStamped>> data_to_interp);

  std::vector<Aircraft> triangulateBogies(std::vector<RangeBearingStamped> friendly_data,
                                          std::vector<RangeVelocityStamped> base_data, Pose friendly_pose);

  std::vector<Aircraft> matchBogies(std::vector<std::pair<std::vector<Aircraft>, long>> bogies_over_time);

  double interpolate(const double& x, const double& x1, const double& y1, const double& x2, const double& y2);

  std::thread friendly_updater;
  std::thread base_updater;
  std::thread bogie_estimator;

  AircraftContainer bogies_;

  void findBogies_();

public:
  Estimator();
  ~Estimator();

  void setSimulator(const std::shared_ptr<Simulator>& simulator);

  std::vector<Aircraft> getBogies();

  GlobalOrd transformBogietoGlobal(Pose friendly_pose, RangeBearingStamped relative_pos);
};

#endif