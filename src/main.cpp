/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Add information here
 *
 *  @author Lucien Morey 12904090
 *  @date 24/5/2020
 */
#include <thread>
#include <vector>
#include "simulator.h"

#include "pure_pursuit.h"
#include "estimator.h"
#include "time_planner.h"
#include "controller.h"

int main(void)
{
  // Create a shared pointer for classes
  std::shared_ptr<Simulator> sim(new Simulator());
  Controller controller;

  // begin aircraft control
  controller.begin(sim);

  return 0;
}
