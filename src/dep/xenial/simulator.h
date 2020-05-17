/*! @file
 *
 *  @brief Simulator for assignment 3.
 *
 *  This class defines the set of methods used to instansiate a simulation of
 *  aircrafts flying around a given airspace. 
 *
 *  @author arosspope
 *  @date 24-08-2018
*/
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <opencv2/opencv.hpp>
#include <atomic>
#include "types.h"  // Provides access to the 'GlobalOrd', and 'Aircraft' types
#include "timer.h"
#include <thread>
#include <random>

class Simulator
{
public:
  /* Declare Public Constants */
  static const double V_TERM;                   /*!< Terminal velocity (m/s) */
  static const unsigned int MAX_G;              /*!< The maximum g-force */
  static const double MAX_V;                    /*!< The maximum velocity (m/s) */
  static const double AIRSPACE_SIZE;            /*!< The airspace size is (AIRSPACE_SIZE x AIRSPACE_SIZE) in metres */
  static const GlobalOrd BSTATION_LOC;          /*!< The location of the base station */
  static const unsigned int BSTATION_REF_RATE;  /*!< The rate (milliseconds) at which the base station produces range to bogie */
  static const unsigned int FRIENDLY_REF_RATE;  /*!< The rate (milliseconds) at which the friendly produces range to bogie */
  const std::string library_version = "Version 20.5.16"; /*!< Library version*/

  /*! @brief Simulator constructor.
   *
   *  Will randomly intialise the position of the bogie and friendly within the airspace.
   */
  Simulator(void);

  /*! @brief Returns the simulation thread.
   *
   *  @note This thread will terminate if `stop()` is called OR, the user has not supplied
   *        control to the friendly aircraft within a timely manner
   */
  std::thread spawn(void);

  /*! @brief Stops the running simulation thread when invoked.
   */
  void stop(void);

  /*! @brief Gets the elapsed time since the simulation started.
   *
   *  @return long - The elapsed time in milliseconds.
   */
  long elapsed(void);

  /*! @brief Return the friendly aircraft's pose.
   *
   *  @return Pose - The aircraft's x, y position (metres) and orientation (radians).
   */
  Pose getFriendlyPose(void);

  /*! @brief Return the friendly aircraft's linear velocity.
   *
   *  @return double - The aircraft's linear velocity (metres/second).
   */
  double getFriendlyLinearVelocity(void);

  /*! @brief Return the friendly aircraft's angular velocity.
   *
   *  @return double - The aircraft's angular velocity (radians/second).
   */
  double getFriendlyAngularVelocity(void);

  /*! @brief Update the linear and angular velocity of the friendly aircraft.
   *
   *  @param linear_velocity The linear velocity of the aircraft (metres/second)
   *  @param angular_velocity The angular velocity of the aircraft (radians/second)
   *
   *  @return bool - Will return false if the calculated gforce is more than 6G's OR if
   *                 the linear velocity is less than the terminal OR the linear velocity is more than the max.
   */
  bool controlFriendly(double linear_velocity, double angular_velocity);

  /*! @brief Returns the range from the base station and velocity of bogies.
   *
   *  @return RangeVelocityStamped - The range from bogie to base (metres), and the velocity of bogie (m/s) with a timestamp (milliseconds).
   *  @note This function will perform a sleep(BASE_STATION_REFRESH_RATE), to enforce
   *        the refresh rate of the friendly aircraft.
   *  @note The timestamp uses the simulation epoch timer as a reference.
   */
  std::vector<RangeVelocityStamped> rangeVelocityToBogiesFromBase(void);

  /*! @brief Returns the range from the friendly aircraft to the bogie.
   *
   *  @return RangeStamped - The range and bearing from friendly to bogies (metres) (rad), with a timestamp (milliseconds).
   *  @note This function will perform a sleep(FRIENDLY_REFRESH_RATE), to enforce
   *        the refresh rate of the friendly aircraft.
   *  @note The timestamp uses the simulation epoch timer as a reference.
   */
  std::vector<RangeBearingStamped> rangeBearingToBogiesFromFriendly(void);

  /*! @brief Will render test aircraft by poses supplied within the airspace.
   *
   *  Callers may use this method to draw additional aircraft within the airspace
   *  for testing purposes. It will stay on screen for 1 second.
   *
   *  @param pose The pose to display.
   */
  void testPose(std::vector<Pose> poses);

  /*! @brief Calculates the euclidean distance between two coordiantes.
   *
   *  @param o1 The first coordinate.
   *  @param o2 The second coordinate.
   *  @return double - The distance between the two coordiantes.
   */
  double distance(GlobalOrd o1, GlobalOrd o2);

private:
  AircraftContainer friendly_, bogies_, test_; /*!< Thread safe container for the aircrafts */
  unsigned int pixel_map_size_;               /*!< Pixel size of the airspace */
  cv::Mat airspace_;                          /*!< The opencv image of the airspace */
  Timer epoch_;                               /*!< Keeps track of time since the invocation of the `run()` thread */
  Timer watchdog_;                            /*!< Keeps track of time since last aircraft control */
  std::atomic<bool> renderTest_;              /*!< Used by the run thread to determine when to render the test aircraft */
  std::atomic<bool> stop_;                    /*!< Used by the simulation thread to know when to stop */
  std::atomic<bool> intercepted_;             /*!< Used to determine when we have successfully trailed the bogie for more than 2 seconds */
  std::default_random_engine* generator_;     /*!< Pointer to the random number generator */
  std::uniform_real_distribution<double> *distribution_; /*!< The pointer to the distributon */
  std::atomic<unsigned int> tally_;            /*!< Number of bogies intercepted within 5 minutes */

  /*! @brief Simulator program that renders the world, updating the aircrafts' position.
   */
  void simulate(void);

  /*! @brief Redraws all actors within the simulation.
   */
  void updateDisplay(void);

  /*! @brief Will draw an aircraft onto the airspace.
   *
   *  @param a The aircraft to draw.
   *  @param colour The colour of the aircraft.
   */
  void drawAircraft(Aircraft a, cv::Scalar colour);

  /*! @brief Will draw the trail of an aircraft.
   *
   *  @param trail The aircraft trail to draw.
   *  @param colour The colour of the trail.
   */
  void drawTrail(std::vector<GlobalOrd> trail, cv::Scalar colour);

  /*! @brief State machine for the bogie aircraft. Called by the run thread.
   */
  void bogieStateMachine(Aircraft &a);

  /*! @brief Update the position of an aircraft within the world.
   *
   *  @param a A reference to the aircraft to update.
   */
  void updateAircraftPosition(Aircraft &a);

  /*! @brief Converts a Global coordinate to a pixel coordinate
   *
   *  @param ord The coordinate to convert.
   *  @return Point - The converted point.
   *  @note The coordinate x & y will round to the nearest 1 decimal place.
   */
  cv::Point convertToPoint(GlobalOrd ord);

  /*! @brief Ensure that an angle is between (0 - 2pi).
   *
   *  @param theta The angle to normalise (radians)
   *  @return double - The normalised angle (radians).
   */
  double normaliseAngle(double theta);

  /*! @brief Generates a goal heading for the bogie to head towards.
   *
   *  @param currentPos The current position of the bogie.
   *  @return double - The heading (radians).
   */
  double generateGoalTheta(GlobalOrd currentPos);

  /*! @brief Create new bogie
   *
   */
  void createBogie();

};

#endif // SIMULATOR_H
