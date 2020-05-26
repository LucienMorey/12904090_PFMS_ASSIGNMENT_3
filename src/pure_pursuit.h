#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "path_tracker.h"
#include <algorithm>

class PurePursuit : public PathTracker
{
private:
  const double MAX_CURVATURE = MAX_ANGLE_VELOCITY / MIN_LINEAR_VELOCITY;
  double gamma = MAX_CURVATURE;

  /**
   * @brief return the sign of a double precision floating point number
   *
   * @param num double - number for testing
   * @return double - will return 1.0 if the number tested is greater than or equal to 0 else it returns -1.0
   */
  double sign(double num);

  /**
   * @brief calculate the point of intersection between a circle and line segment. If there are two points of
   * intersection the point furthest along will be returned. If there are no points of interception an empty vector will
   * be returned.
   *
   * @param segment_begin - GlobalOrd, Point beginning of line segment
   * @param segment_end - GlobalOrd, Point end of line segment
   * @param circle_origin - GlobalOrd, Point centre of circle
   * @param circle_radius - double, radius of circle
   * @return std::vector<GlobalOrd> - vector containing point of interception
   */
  std::vector<GlobalOrd> lineCircleIntercept(GlobalOrd segment_begin, GlobalOrd segment_end, GlobalOrd point_checking,
                                             double circle_radius);

  /**
   * @brief Determine the point along a line segment that is closest to a test position. It will either return a point
   * creating a perpendicular line from the segment to the test point or will return either the beginning or end of
   * segment point.
   *
   * @param segment_begin - GlobalOrd, point beginning of segment
   * @param segment_end - GlobalOrd, point end of segment
   * @param point_checking - GlobalOrd, point for checking
   * @return GlobalOrd, closest point along the line to the point being cheked
   */
  GlobalOrd closestPointAlongSegmentFromPoint(GlobalOrd segment_begin, GlobalOrd segment_end, GlobalOrd point_checking);

public:
  /**
   * @brief Construct a new Pure Pursuit object
   *
   */
  PurePursuit(){};

  /**
   * @brief Destroy the Pure Pursuit object
   *
   */
  ~PurePursuit(){};

  /**
   * @brief Track goal pose parsed to function. The pure pursuit will rotate until it meets a specified angle the nwill
   * calculate and pursue a small curvature to maximise linear velocity and minimise angular velocity to a target. it
   * requries an initial and goal pose to create a path. the current pose and current velocity will assist and tracking
   * a point along the path until the end
   *
   * @param current_pose - Pose, current pose of Aircraft
   * @param current_velocity - double, current linear velocity of aircraft
   * @param initial_pose - Pose, initial pose of Aircraft along the path segment
   * @param target_pose - Pose, final Desired Pose. The pure pursuit algorithm will only be able to track position and
   * not full pose
   * @return Twist_t - struct containing vX,vY and vZ (Yaw) for aircraft to undertake
   */
  Twist_t track(const Pose& current_pose, double current_velocity, const Pose& initial_pose, const Pose& target_pose);
};

#endif