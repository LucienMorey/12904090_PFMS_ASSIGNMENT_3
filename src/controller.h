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

  void plannerThread();
  void controlThread();

  std::vector<std::thread> threads;

public:
  Controller();
  ~Controller();
  void begin(std::shared_ptr<Simulator> sim);
};

#endif