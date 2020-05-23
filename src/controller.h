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
  Planner* planner_;

  void plannerThread();
  void controlThread();

  std::vector<std::thread> threads;
  std::mutex mx;

public:
  Controller(std::shared_ptr<Simulator> sim);
  ~Controller();
  void begin();
};

#endif