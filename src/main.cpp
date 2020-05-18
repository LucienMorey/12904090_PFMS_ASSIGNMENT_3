/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Add information here
 *
 *  @author {TODO: Your student name + id}
 *  @date {TODO}
 */
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"

#include "pure_pursuit.h"
#include "estimator.h"

// For example purposes only, this thread attmps to get the friendly aircraft's
//(red triangle) pose every 4 seconds. It plots this pose on the
// simulation (blue triangle) which stays on the image for 1 second, as per the
//'testPose()' documentation in the simualtor class.
void exampleThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<Estimator>& estimator)
{
  while (true)
  {
    // Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    std::vector<Pose> poses;
    std::vector<Aircraft> bogies = estimator->getBogies();

    for (auto bogie : bogies)
    {
      poses.push_back(bogie.pose);
    }
    sim->testPose(poses);
    poses.clear();

    // std::cout << bogies.size() << std::endl;

    // if (bogies.size() > 2)
    // {
    //   std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
    //   std::cout << "Friendly {x, y, orientation}:" << std::endl;
    //   std::cout << "  - x: " << bogies.front().pose.position.x << "m" << std::endl;
    //   std::cout << "  - y: " << bogies.front().pose.position.y << "m" << std::endl;
    //   std::cout << "  - orient: " << bogies.front().pose.orientation * 180 / M_PI << " deg" << std::endl <<
    //   std::endl; std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
  }
}

// This thread will simply get the current velocity and feed it back into
// controlled velocity, at the designated minimum time required (watchdog time) refer
//'controlFriendly()' documentation in the simualtor class.
void controlThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<path_tracker>& tracker)
{
  while (true)
  {
    // Feed the watchdog control timer
    Twist_t next_twist = tracker->track(sim->getFriendlyPose(), Pose{ { -2000.00, 100 }, 2.0 });

    sim->controlFriendly(next_twist.vY, next_twist.vZ);
    // sim->testPose(std::vector<Pose>(1, { { -2000.0, 100.0 }, 2.0 }));

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

int main(void)
{
  std::vector<std::thread> threads;

  // Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  std::shared_ptr<path_tracker> tracker(new PurePursuit());
  std::shared_ptr<Estimator> estimator(new Estimator());
  estimator->setSimulator(sim);
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim, tracker));
  threads.push_back(std::thread(exampleThread, sim, estimator));

  // Join threads and begin!
  for (auto& t : threads)
  {
    t.join();
  }

  return 0;
}
