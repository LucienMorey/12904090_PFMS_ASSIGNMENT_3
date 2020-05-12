/*! @file
 *
 *  @brief A library of simple types used by the simulator class.
 *
 *  @author arosspope
 *  @date 24-08-2018
*/
#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <mutex>
#include <cmath>
#include "timer.h"

static const double float_comparison_tolerence =0.1;

/* Enum for the Aicraft state machine */
enum AircraftState {
  BS_START=0,
  BS_STRAIGHT=1,
  BS_TURNING=2,
  BS_RECOVERY=3,
  BS_UNKNOWN=-1,
};

struct GlobalOrd {
  double x;   /*!< x coordinate within global map (m) */
  double y;   /*!< y coordinate within global map (m) */

  /*!< Implementing the operator '==' for this struct */
  bool operator== (const GlobalOrd &o1){
    return ((std::abs(this->x - o1.x)<float_comparison_tolerence) && (std::abs(this->y - o1.y)<float_comparison_tolerence));
  }
};

struct Pose {
  GlobalOrd position; /*!< Global position (metres) */
  double orientation; /*!< Orientation (radians) */
};

struct RangeBearingStamped { /*!< Contains a timestamped range/bearing reading */
  double range;       /*!< The range (distance reading) in metres */
  double bearing;     /*!< The bearing (bearing reading) in radians */
  long timestamp;     /*!< Timestamp (milliseconds) */
};

struct RangeVelocityStamped { /*!< Contains a timestamped range/bearing reading */
  double range;       /*!< The range (distance reading) in metres */
  double velocity;       /*!< Linear velocity (metres/second) */
  long timestamp;     /*!< Timestamp (milliseconds) */
};


struct Aircraft {
  Pose pose;                    /*!< Global position and orientation within the airspace */
  std::vector<GlobalOrd> trail; /*!< To display where the aircraft has been */
  double linear_velocity;       /*!< Linear velocity (metres/second) */
  double angular_velocity;      /*!< Angular velocity (radians/second). (+) Counter clockwise, (-) Clockwise. */
  Timer timer;                  /*!< Used to keep track of elapsed time. */
  AircraftState state;          /*!< Aircraft state during operation. */
  Pose currentGoalPose;                /*!< Global goal position and orientation within the airspace. */
  Pose previousGoalPose;
};

struct AircraftContainer { /*!< A thread safe container for the Aircraft type */
  std::vector<Aircraft> a;
  std::mutex access;
};
#endif // TYPES_H
