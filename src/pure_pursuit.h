#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "path_tracker.h"

class PurePursuit : public path_tracker
{
private:
  double sign(double num);

  const double MAX_CURVATURE = MAX_ANGLE_VELOCITY / MIN_LINEAR_VELOCITY;
  double gamma = MAX_CURVATURE;

  std::vector<GlobalOrd> lineCircleIntercept(GlobalOrd segment_begin, GlobalOrd segment_end, GlobalOrd point_checking,
                                             double circle_radius);
  GlobalOrd closestPointAlongSegmentFromPoint(GlobalOrd segment_begin, GlobalOrd segment_end, GlobalOrd point_checking);

public:
  PurePursuit(){};
  ~PurePursuit(){};
  Twist_t track(const Pose& current_pose, double current_velocity, const Pose& initial_pose, const Pose& target_pose);
};

#endif