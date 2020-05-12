/*! @file
 *
 *  @brief A class for measuring periods of time.
 *
 *  This class contains routines for measuring a period of
 *  time given a reference point on the system clock.
 *
 *  @author arosspope
 *  @date 11-09-2017
*/
#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer
{
public:
  /*! @brief Constructor for the timer.
   *
   *  Upon creation of the object, the reference time is set.
   *  Or more generally, 'the timer is started'.
   */
  Timer();

  /*! @brief Resets the timer.
   *
   *  Will update the reference time to the current
   *  value of the system clock.
   */
  void reset();

  /*! @brief Returns the elapsed period in milliseconds.
   *
   *  Calculates the difference between the current time
   *  and the reference time point.
   *
   *  @return long - The elapsed period in (ms).
   */
  long elapsed() const;
private:
  typedef std::chrono::high_resolution_clock clock_;
  std::chrono::time_point<clock_> ref_;
};

#endif // TIMER_H
