#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "path_tracker.h"

class PurePursuit : public path_tracker
{
private:
  double ANGULAR_KP = 0.3;

  double pi2Topi(double angle);
  double sign(double num);

  const double MAX_CURVATURE = MAX_ANGLE_VELOCITY / MIN_LINEAR_VELOCITY;
  double gamma = MAX_CURVATURE;

  std::vector<GlobalOrd> line_circle_intercept(GlobalOrd segment_begin, GlobalOrd segment_end, GlobalOrd point_checking,
                                               double circle_radius);
  GlobalOrd point_line_perpendicular_d(GlobalOrd segment_begin, GlobalOrd segment_end, GlobalOrd point_checking);

public:
  PurePursuit(){};
  ~PurePursuit(){};
  Twist_t track(const Pose& current_pose, double current_velocity, const Pose& initial_pose, const Pose& target_pose);
};

#endif