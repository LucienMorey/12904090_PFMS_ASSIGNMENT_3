#include "pure_pursuit.h"

double PurePursuit::sign(double num)
{
  if (num >= 0)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

Twist_t PurePursuit::track(const Pose& current_pose, double current_velocity, const Pose& initial_pose,
                           const Pose& target_pose)
{
  // calculate the delta x and y between the goal pose and the current pose
  double delta_dx_end = target_pose.position.x - current_pose.position.x;
  double delta_dy_end = target_pose.position.y - current_pose.position.y;

  // calculate the overall heading error
  double alpha = atan2(delta_dy_end, delta_dx_end);
  double heading_error = alpha - atan2(sin(current_pose.orientation), cos(current_pose.orientation));
  // absolute heading error should never be greater than pi. if it is then wrap the angle
  if (heading_error > M_PI)
  {
    heading_error = heading_error - 2 * M_PI;
  }
  else if (heading_error < -M_PI)
  {
    heading_error = 2 * M_PI + heading_error;
  }

  // if the heading error is too large then turn until purepursuit is viable
  if (fabs(heading_error) < LARGE_ANGULAR_TOLERANCE)
  {
    // Determine the look_ahead
    double look_ahead = current_velocity;

    std::vector<GlobalOrd> point_to_track;

    // the point to track will be the interception between the linear path and a circle with radius = look ahead around
    // the robot
    // there can be up to two interceptions. in this case the furthest interception will be considered
    point_to_track =
        lineCircleIntercept(initial_pose.position, target_pose.position, current_pose.position, look_ahead);

    // if there was no interception then the robot should get back to the path as quickly as possible and will track the
    // closest interception
    if (point_to_track.size() == 0)
    {
      // find point at line at perpendicular distance
      GlobalOrd closest_interception =
          closestPointAlongSegmentFromPoint(initial_pose.position, target_pose.position, current_pose.position);
      point_to_track.push_back(closest_interception);
    }

    // calculate the perpendicular distance between the current position and the path
    double a = -tan(current_pose.orientation);
    double b = 1;
    double c = tan(current_pose.orientation) * current_pose.position.x - current_pose.position.y;
    double x = fabs(a * point_to_track.front().x + b * point_to_track.front().y + c) / sqrt(pow(a, 2) + pow(b, 2));

    // note the side of the path that the friendly occupies
    int side = sign(sin(current_pose.orientation) * (point_to_track.front().x - current_pose.position.x) -
                    cos(current_pose.orientation) * (point_to_track.front().y - current_pose.position.y));

    // calculate curvature
    gamma = 2.0 * x / pow(look_ahead, 2);

    // calculate angular velocity
    double angular_velocity = -sign(side) * sqrt(MAX_G * G * fabs(gamma));
    // ensure it remains within maximum bounds
    angular_velocity = fmax(angular_velocity, -MAX_ANGLE_VELOCITY);
    angular_velocity = fmin(angular_velocity, MAX_ANGLE_VELOCITY);

    // calculate linear velocity so that the bogie always flies at 6Gs
    double linear_velocity = fabs(angular_velocity / gamma);
    // cap the linear velocity at the boundaries of the friendly
    linear_velocity = fmin(linear_velocity, MAX_LINEAR_VELOCITY);
    linear_velocity = fmax(linear_velocity, MIN_LINEAR_VELOCITY);

    return Twist_t{ linear_velocity, 0.0, angular_velocity };
  }
  else
  {
    // rotate at maximum angle velocity until within angle allowance
    double angular_velocity = sign(heading_error) * MAX_ANGLE_VELOCITY;

    // constrain angle velocity
    angular_velocity = fmax(angular_velocity, -MAX_ANGLE_VELOCITY);
    angular_velocity = fmin(angular_velocity, MAX_ANGLE_VELOCITY);

    // set linear velocity to min allowable to "rotate on the spot"
    double linear_velocity = MIN_LINEAR_VELOCITY;

    return Twist_t{ linear_velocity, 0.0, angular_velocity };
  }
}

std::vector<GlobalOrd> PurePursuit::lineCircleIntercept(GlobalOrd segment_begin, GlobalOrd segment_end,
                                                        GlobalOrd circle_origin, double circle_radius)
{
  // solve quadratic to determine the intersection points of line parametrically

  // create vector joining the beginning and end of the line segment
  GlobalOrd line_segment = { (segment_end.x - segment_begin.x), (segment_end.y - segment_begin.y) };

  // create vector joining the circle centre and the beginning of the line
  // segment
  GlobalOrd circle_to_segment_begin = { (segment_begin.x - circle_origin.x), (segment_begin.y - circle_origin.y) };

  // the dot product of the line segment and the line segment
  double a = line_segment.x * line_segment.x + line_segment.y * line_segment.y;

  // 2 * the dot product of line segment vector and circle_to line beginning
  // vector
  double b = 2 * (circle_to_segment_begin.x * line_segment.x + circle_to_segment_begin.y * line_segment.y);

  // circle to line begin dot circle to line begin - r^2
  double c =
      (circle_to_segment_begin.x * circle_to_segment_begin.x + circle_to_segment_begin.y * circle_to_segment_begin.y) -
      circle_radius * circle_radius;

  double discriminant = b * b - 4 * a * c;

  double t1;
  double t2;

  std::vector<GlobalOrd> intersection_points;

  if (discriminant < 0)
  {
    // no intersection
  }
  else
  {
    discriminant = sqrt(discriminant);
    // solve the quadratic
    t1 = (-b - discriminant) / (2 * a);
    t2 = (-b + discriminant) / (2 * a);

    if ((t1 >= 0) && (t1 <= 1))
    {
      intersection_points.push_back(GlobalOrd{ (segment_begin.x + t1 * (segment_end.x - segment_begin.x)),
                                               (segment_begin.y + t1 * (segment_end.y - segment_begin.y)) });
    }

    // returnt1 intersection
    if (((t2 >= 0) && (t2 <= 1)) && (t2 > t1))
    {
      // returnt2 intersection
      intersection_points.push_back(GlobalOrd{ (segment_begin.x + t2 * (segment_end.x - segment_begin.x)),
                                               (segment_begin.y + t2 * (segment_end.y - segment_begin.y)) });
    }

    // otherwise, no intersection in range of segment
  }

  // if there are two intersection points then keep the point the furthest along the segment
  if (intersection_points.size() == 2)
  {
    if (t1 > t2)
    {
      intersection_points.pop_back();
    }
    else
    {
      intersection_points.pop_back();
      intersection_points.front() = GlobalOrd{ (segment_begin.x + t2 * (segment_end.x - segment_begin.x)),
                                               (segment_begin.y + t2 * (segment_end.y - segment_begin.y)) };
    }
  }

  return intersection_points;
}

GlobalOrd PurePursuit::closestPointAlongSegmentFromPoint(GlobalOrd segment_begin, GlobalOrd segment_end,
                                                         GlobalOrd point_checking)
{
  // implementation of code found here: http://www.fundza.com/vectors/point2line/index.html
  GlobalOrd line_vector = { segment_end.x - segment_begin.x, segment_end.y - segment_begin.y };

  GlobalOrd point_vector = { point_checking.x - segment_begin.x, point_checking.y - segment_begin.y };

  double line_length = sqrt(pow(line_vector.x, 2) + pow(line_vector.y, 2));

  GlobalOrd unit_line_vector = { line_vector.x / line_length, line_vector.y / line_length };

  GlobalOrd scaled_point_vector = { point_vector.x * (1.0 / line_length), point_vector.y * (1.0 / line_length) };

  double t = unit_line_vector.x * scaled_point_vector.x + unit_line_vector.y * scaled_point_vector.y;

  t = fmin(1.0, t);
  t = fmax(0.0, t);

  GlobalOrd nearest_point = { segment_begin.x + t * (segment_end.x - segment_begin.x),
                              segment_begin.y + t * (segment_end.y - segment_begin.y) };

  return nearest_point;
}
