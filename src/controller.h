#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "simulator.h"
#include "pure_pursuit.h"
#include "estimator.h"
#include "time_planner.h"

class Controller
{
private:
  Simulator* sim_;
  PathTracker* tracker_;
  Estimator* estimator_;
  Planner* planner_;

  /**
   * @brief Thread safe function designed to triangulate and plan path to bogies in real time
   *
   */
  void plannerThread();

  /**
   * @brief Thread safe function that will get the lastest path from the planner thread. if the path is valid, it will
   * be tracked with pure pursuit otherswise the friendly will receive a command of min linear velocity to ensure the
   * watchdog is fed
   *
   */
  void controlThread();

  std::vector<std::thread> threads;

public:
  /**
   * @brief Construct a new Controller object
   *
   */
  Controller();

  /**
   * @brief Destroy the Controller object. join all started threads and deloete hanging pointers
   *
   */
  ~Controller();

  /**
   * @brief begin control of Friendly Aircraft in simulation. Function creates all componets and starts threads for
   * proper simulator estimation and control
   *
   * @param sim pointeer to simulator that has been created requiring control
   */
  void begin(std::shared_ptr<Simulator> sim);
};

#endif