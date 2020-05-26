#ifndef TIMEPLANNER_H
#define TIMEPLANNER_H

#include "planner.h"
#include <map>
#include <algorithm>

class TimePlanner : public Planner
{
public:
  /**
   * @brief Construct a new Time Planner object. Subclassed from virtual planner object. considers time
   * to interception when weighing path segments
   *
   */
  TimePlanner();

  /**
   * @brief Destroy the Time Planner:: Time Planner object
   *
   */
  ~TimePlanner();

  /**
   * @brief Take a vector of aircraft where the friendly aircraft is in the first position and create a cost function to
   * determine the most efficient bogie to pursue based on time to interception.
   *
   * @param aircraft - vector of aircraft to be considered. the first aircraft in the vector should be the friendly
   * aircraft. any aircraft after will be considered to be bogies
   */
  void plan(std::vector<Aircraft> aircraft);
};

#endif
