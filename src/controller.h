#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "simulator.h"
#include "pure_pursuit.h"
#include "estimator.h"
#include "distance_planner.h"
#include "time_planner.h"

class Controller
{
private:
  Simulator* sim_;
  path_tracker* tracker_;
  Estimator* estimator_;
  TimePlanner* planner_;
  std::vector<Pose> poses;

  double trajectory_time = 0.0;

  std::chrono::steady_clock::time_point time_point_last_scan = std::chrono::steady_clock::now();

  void plannerThread();
  void controlThread();

  std::vector<std::thread> threads;
  std::mutex mx;
  std::condition_variable cond;

public:
  Controller(std::shared_ptr<Simulator> sim);
  ~Controller();
  void begin();
};

#endif